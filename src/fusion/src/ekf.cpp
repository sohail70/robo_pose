#include<fusion/ekf.hpp>
#include<fusion/constant_heading_rate.hpp>
namespace Filter{
    template<typename velType , typename imuType>
    Ekf<velType,imuType>::Ekf(std::unique_ptr<MotionModel> motion_model_ ,
             std::shared_ptr<VelocitySource<velType>> velocity_source_ ,
             std::shared_ptr<ImuSource<imuType>> imu_source_):
    Fusion(std::move(motion_model_)) ,velocity_source_(velocity_source_) , imu_source_(imu_source_)
    {
        std::cout<<"Ctor of Ekf \n";
    }

    template<typename velType , typename imuType>
    void Ekf<velType,imuType>::initialize()
    {
        // initialize dimensions of the state space
        int num_states_ = 5; //[x,y,yaw,x_dot,yaw_dot]
        int num_inputs_ = 2;
        //for now 2 data is good --> i might have to figure out how to incorporate a_x and a_y data as well which might gives me maybe x and y by double integrating or just use a_x and integrate it once to get x_dot --> the realtion ship is maybe nonlinear and i need to take care of it
        int num_obs_ = 1; // my imu gives 9 data (3 for orientation - 3 for angular velocity and 3 for accelration in x,y,z direction but im using only yaw data - yaw_dot or angular velocity in z direction)

        A = Eigen::MatrixXd(num_states_ , num_states_);
        B = Eigen::MatrixXd(num_states_ , num_inputs_);
        // Q = Eigen::MatrixXd(num_states_ , num_states_);
        Q.setZero(num_states_ , num_states_);
        // the following number i get from /odom topic of the diff drive controller  which is 6by6 cov matrix for x,y,z,roll,pitch.yaw and in the twist their dots also can be found. i got what ever is related to my state space
        Q(0,0) = 1.0e-5;
        Q(1,1) = 1.0e-5;
        Q(2,2) = 1.0e-3;
        Q(3,3) = 1.0e-5;
        Q(4,4) = 1.0e-3;
        H = Eigen::MatrixXd(num_obs_ , num_states_);
        // H<< 0,0,1,0,0,0,0,0,0,1; //we extract the yaw dot from imu . lets not use the oienation (yaw) becase i think that data is redudnat and not available directly! but im not sure
        H<< 0,0,0,0,1; //we extract the yaw dot from imu . lets not use the oienation (yaw) becase i think that data is redudnat and not available directly! but im not sure


        // R = Eigen::MatrixXd(num_obs_ , num_obs_);
        R.setZero(num_obs_ , num_obs_); //for yaw and yaw_rate
        R(0,0) = 4.0e-8;
        // P = Eigen::MatrixXd(num_states_ , num_states_);
        P.setZero(num_states_,num_states_);
        I = Eigen::MatrixXd::Identity(num_states_ , num_states_);

        X.setZero(num_states_,1);
    }

    template<typename velType , typename imuType>
    void Ekf<velType,imuType>::predict(const rclcpp::Time& current_time_)
    {
        auto cmd = velocity_source_->getVelocity();
        motion_model_->setVelAndAngVelFromTwist(cmd);
        cmd.angular.z = X(4,0); // from the updated yaw_dot from ekf's update
        motion_model_->setVelAndAngVelFromTwist(cmd);
        motion_model_->update(current_time_);
        rclcpp::Duration dt = motion_model_->getDt();
        double x = dynamic_cast<Filter::ConstantHeadingRate*>(motion_model_.get())->getPosition().x;
        double y = dynamic_cast<Filter::ConstantHeadingRate*>(motion_model_.get())->getPosition().y;
        double v = dynamic_cast<Filter::ConstantHeadingRate*>(motion_model_.get())->getVelocity().x_dot;
        double w = dynamic_cast<Filter::ConstantHeadingRate*>(motion_model_.get())->getAngularVelocity().yaw_dot;
        double yaw = dynamic_cast<Filter::ConstantHeadingRate*>(motion_model_.get())->getAngle().yaw;
        A(0,0) = 1; A(0,1) = 0; A(0,2) = -v*sin(yaw)*dt.seconds(); A(0,3)= cos(yaw)*dt.seconds(); A(0,4) = 0;  
        A(1,0) = 0; A(1,1) = 1; A(1,2) =  v*cos(yaw)*dt.seconds(); A(1,3)= sin(yaw)*dt.seconds(); A(1,4) = 0;  
        A(2,0) = 0; A(2,1) = 0; A(2,2) =  1            ; A(2,3)= 0          ; A(2,4) = dt.seconds();  
        A(3,0) = 0; A(3,1) = 0; A(3,2) =  0            ; A(3,3)= 1          ; A(3,4) = 0;  
        A(4,0) = 0; A(4,1) = 0; A(4,2) =  0            ; A(4,3)= 0          ; A(4,4) = 1;  
        // Are you tempted to use the A matrix to update the X? don't :) use the original non linear model to update the States. use A only in P calculations
        X(0,0) = x;
        X(1,0) = y;
        X(2,0) = yaw;
        X(3,0) = v;
        X(4,0) = w;
        
        P = A*P*A.transpose() + Q;
        // std::cout<<"V_x: "<<v<<" W: "<<w<<" Yaw:"<< yaw <<" dt:" <<dt.seconds()<<"\n";
        // std::cout<<A<<"\n \n \n";
        // std::cout<<X<<"\n \n \n"; 
        // std::cout<<P<<"\n \n \n";



    }

    template<typename velType , typename imuType>
    void Ekf<velType,imuType>::update()
    {
        // std::cout<<"In the update \n";
        Eigen::MatrixXd measurement_prediction_ = H*X;
        // std::cout<<H<<"\n \n \n ";
        // std::cout<<X<<"\n \n \n ";
        // std::cout<<measurement_prediction_<<"\n"; //I only get data when i give angular velocity
        int num_obs_ = 1;
        Eigen::MatrixXd imu_measurement_(num_obs_ , 1);
        imu_measurement_<< imu_source_->getImuData().angular_velocity.z ;
        Eigen::MatrixXd measurement_residual_ = imu_measurement_ - measurement_prediction_;
        // std::cout<< imu_measurement_<<"  "<< measurement_prediction_ <<" "<<measurement_residual_<<"\n";
        // std::cout<<imu_source_->getImuData().angular_velocity.z<<"\n";
        Eigen::MatrixXd innovation_covariance_ = H*P*H.transpose() + R;
        Eigen::MatrixXd kalman_gain_ = P*H.transpose()*innovation_covariance_.inverse();
        // std::cout<<innovation_covariance_<<"\n";
        // std::cout<<"KALMAN GAIN:\n"<<kalman_gain_<<"\n";
        // std::cout<<"K*res:\n"<<kalman_gain_*measurement_residual_<<"\n";
        X = X + kalman_gain_*measurement_residual_;
        // P = (I-kalman_gain_*H)*P*(I-kalman_gain_*H).transpose() + kalman_gain_*R*kalman_gain_.transpose(); // or use P = (I-K*H)*P
        P = (I-kalman_gain_*H)*P;
        // std::cout<<"X: \n"<<X.transpose()<<"\n \n \n";
        // std::cout<<"P: \n"<<P<<"\n \n \n ";
    }

    template<typename velType, typename imuType>
    Eigen::MatrixXd Ekf<velType,imuType>::getStates()
    {
        return X;
    }


    template class Ekf<geometry_msgs::msg::Twist, sensor_msgs::msg::Imu>;

} //namespace Filter