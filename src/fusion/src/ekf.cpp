#include<fusion/ekf.hpp>
#include<fusion/constant_heading_rate.hpp>

#include<tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/quaternion.hpp>
namespace Filter{

    Ekf::Ekf()
    {
        std::cout<<"default Ctor of Ekf \n";
    }
    
    Ekf::Ekf(std::unique_ptr<MotionModel> motion_model_):
    Fusion(std::move(motion_model_)) 
    {
        std::cout<<"Ctor of Ekf \n";
    }

    void Ekf::setStates(StateSpace* states_)
    {
        this->states_ = states_;
    }

    void Ekf::setMotionModel(std::unique_ptr<MotionModel> motion_model_)
    {
        this->motion_model_ = motion_model_ ? std::move(motion_model_) : nullptr;
    }

    void Ekf::initialize()
    {
        // initialize dimensions of the state space
        int num_states_ = states_->getStates().size(); //[x,y,yaw,x_dot,yaw_dot,x_ddot]
        int num_inputs_ = 2;
        //for now 2 data is good --> i might have to figure out how to incorporate a_x and a_y data as well which might gives me maybe x and y by double integrating or just use a_x and integrate it once to get x_dot --> the realtion ship is maybe nonlinear and i need to take care of it
        int num_obs_ = 3; // my imu gives 9 data (3 for orientation - 3 for angular velocity and 3 for accelration in x,y,z direction but im using only yaw data - yaw_dot or angular velocity in z direction)
       
        Q.setZero(num_states_ , num_states_);
        // the following number i get from /odom topic of the diff drive controller  which is 6by6 cov matrix for x,y,z,roll,pitch.yaw and in the twist their dots also can be found. i got what ever is related to my state space
        Q(0,0) = 1.0e-5;
        Q(1,1) = 1.0e-5;
        Q(2,2) = 1.0e-3;
        Q(3,3) = 1.0e-5;
        Q(4,4) = 1.0e-3;
        Q(5,5) = 1.0e-8;
        H = Eigen::MatrixXd(num_obs_ , num_states_);
        // H<< 0,0,1,0,0,0,0,0,0,1; //we extract the yaw dot from imu . lets not use the oienation (yaw) becase i think that data is redudnat and not available directly! but im not sure
        H<<0,0,1,0,0,0 ,0,0,0,0,1,0 ,0,0,0,0,0,1; //we extract the yaw dot from imu . lets not use the oienation (yaw) becase i think that data is redudnat and not available directly! but im not sure


        // R = Eigen::MatrixXd(num_obs_ , num_obs_);
        R.setZero(num_obs_ , num_obs_); //for yaw and yaw_rate
        R(0,0) = 4.0e-8;
        R(1,1) = 4.0e-8;
        R(2,2) = 2.0e-4;
        // P = Eigen::MatrixXd(num_states_ , num_states_);
        P.setZero(num_states_,num_states_);
        I = Eigen::MatrixXd::Identity(num_states_ , num_states_);

    }

    void Ekf::predict(const rclcpp::Time& current_time_)
    {
        A = motion_model_->update(current_time_);
        P = A*P*A.transpose() + Q;
        // std::cout<<"P0: \n"<<P<<"\n";
        // std::cout<<"V_x: "<<states_->getStates()["x_dot"]<<" W: "<<states_->getStates()["yaw_dot"]<<" Yaw:"<< states_->getStates()["yaw"] <<" dt:" <<dt.seconds()<<"\n";
    }

    void Ekf::update()
    {
        autodiff::VectorXreal& X = states_->getStates();
        // std::cout<<"In the update \n";
        autodiff::MatrixXreal measurement_prediction_ = H*X;
        // std::cout<<H<<"\n \n \n ";
        // std::cout<<X<<"\n \n \n ";
        // std::cout<<measurement_prediction_<<"\n"; //I only get data when i give angular velocity
        int num_obs_ = 3;
        autodiff::MatrixXreal imu_measurement_(num_obs_ , 1);

        // tf2::Quaternion quaternion;
        // tf2::fromMsg(imu_source_->getImuData().orientation, quaternion);
            
        //     // Convert quaternion to euler angles directly
        // double roll, pitch, yaw;
        // tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        // imu_measurement_<<yaw, imu_source_->getImuData().angular_velocity.z ,imu_source_->getImuData().linear_acceleration.x ;
        imu_measurement_<<0,0,0;
        autodiff::MatrixXreal measurement_residual_ = imu_measurement_ - measurement_prediction_;
        // std::cout<< imu_measurement_<<"  "<< measurement_prediction_ <<" "<<measurement_residual_<<"\n";
        // std::cout<<imu_source_->getImuData().angular_velocity.z<<"\n";
        autodiff::MatrixXreal innovation_covariance_ = H*P*H.transpose() + R;
        autodiff::MatrixXreal kalman_gain_ = P*H.transpose()*innovation_covariance_.inverse();
        // std::cout<<innovation_covariance_<<"\n";
        // std::cout<<"KALMAN GAIN:\n"<<kalman_gain_<<"\n";
        // std::cout<<"K*res:\n"<<kalman_gain_*measurement_residual_<<"\n";
        X = X + kalman_gain_*measurement_residual_;
        // P = (I-kalman_gain_*H)*P*(I-kalman_gain_*H).transpose() + kalman_gain_*R*kalman_gain_.transpose(); // or use P = (I-K*H)*P
        P = (I-kalman_gain_*H)*P;


    }

    Eigen::MatrixXd Ekf::getStates()
    {
        Eigen::MatrixXd X;
        X.setZero(5,1);
        X<<states_->getStates()[states_->getStateOrder()["x"]].val(),
           states_->getStates()[states_->getStateOrder()["y"]].val(),
           states_->getStates()[states_->getStateOrder()["yaw"]].val(),
           states_->getStates()[states_->getStateOrder()["x_dot"]].val(),
           states_->getStates()[states_->getStateOrder()["yaw_dot"]].val();
           states_->getStates()[states_->getStateOrder()["x_ddot"]].val();
        
        return X;
    }



} //namespace Filter