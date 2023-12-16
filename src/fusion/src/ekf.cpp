#include<fusion/ekf.hpp>
#include<fusion/constant_heading_rate.hpp>
namespace Filter{
    Ekf::Ekf(std::unique_ptr<MotionModel> motion_model_):Fusion(std::move(motion_model_))
    {
        std::cout<<"Ctor of Ekf \n";
    }

    void Ekf::initialize()
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
    }

    void Ekf::predict(const rclcpp::Time& current_time_)
    {
        motion_model_->update(current_time_);
        double dt = current_time_.seconds() - motion_model_->getPrevTime().seconds();
        double v = dynamic_cast<Filter::ConstantHeadingRate*>(motion_model_.get())->getVelocity().x_dot;
        double w = dynamic_cast<Filter::ConstantHeadingRate*>(motion_model_.get())->getAngularVelocity().yaw_dot;
        double yaw = dynamic_cast<Filter::ConstantHeadingRate*>(motion_model_.get())->getAngle().yaw;
        A(0,0) = 1; A(0,1) = 0; A(0,2) = -v*sin(yaw)*dt; A(0,3)= cos(yaw)*dt; A(0,4) = 0;  
        A(1,0) = 0; A(1,1) = 1; A(1,2) =  v*cos(yaw)*dt; A(1,3)= sin(yaw)*dt; A(1,4) = 0;  
        A(2,0) = 0; A(2,1) = 0; A(2,2) =  1            ; A(2,3)= 0          ; A(2,4) = dt;  
        A(3,0) = 0; A(3,1) = 0; A(3,2) =  0            ; A(3,3)= 1          ; A(3,4) = 0;  
        A(4,0) = 0; A(4,1) = 0; A(4,2) =  0            ; A(4,3)= 0          ; A(4,4) = 1;  

        P = A*P*A.transpose() + Q;
        

    }

    void Ekf::update()
    {

    }

} //namespace Filter