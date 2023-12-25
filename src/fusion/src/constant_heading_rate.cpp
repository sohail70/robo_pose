
#include "fusion/constant_heading_rate.hpp"
namespace Filter{
    ConstantHeadingRate::ConstantHeadingRate()
    {
        std::cout<<"Ctor of constant heading rate \n";
        init();
    }

    void ConstantHeadingRate::init()
    {
        previous_time_ = rclcpp::Clock().now();
    }

    autodiff::VectorXreal ConstantHeadingRate::update(const autodiff::VectorXreal& state)
    {
        // dt_ = current_time_ - previous_time_;

            // position_.x = position_.x + velocity_.x_dot*cos(angle_.yaw)*dt_.seconds();
            // position_.y = position_.y + velocity_.x_dot*sin(angle_.yaw)*dt_.seconds();
            // angle_.yaw = angle_.yaw + angular_velocity_.yaw_dot*dt_.seconds();
            // normalizeAngle(angle_.yaw);
            // previous_time_ = current_time_;
        // auto& state_ = states_->getStates(); //don't forget the & because we don't wanna update a copy of the states!
        // state_["x"] += state_["x_dot"]*cos(state_["yaw"])*dt_.seconds();
        // state_["y"] += state_["x_dot"]*sin(state_["yaw"])*dt_.seconds();
        // state_["yaw"] += state_["yaw_dot"]*dt_.seconds();
        // normalizeAngle(state_["yaw"]);
        
        // autodiff::VectorXreal new_state_(states_->states_.size());
        // int index = 0;
        // for(auto it = states_->getStateOrder().begin() ; it!= states_->getStateOrder().end() ; ++it)
        // {
        //     new_state_(index) = states_->getStates()[*it];
        //     // std::cout<<"S:"<<state_(index)<<"\n";
        //     index++;
        // }
        // // previous_time_ = current_time_;
        // std::cout<<"herhre \n ";
        // return new_state_;
            autodiff::VectorXreal newState(5);
            autodiff::real dt = autodiff::real(dt_.seconds()); 
            newState(0) = state(0) + state(3) * cos(state(2)) * dt; // Update x
            newState(1) = state(1) + state(3) * sin(state(2)) * dt; // Update y
            newState(2) = state(2) + state(4) * dt;                 // Update yaw angle
            newState(3) = state(3);                                 // Velocity remains constant
            newState(4) = state(4);                                 // Yaw rate remains constant

            return newState;


    }

    Eigen::MatrixXd ConstantHeadingRate::getJacobian()
    {
        Eigen::MatrixXd A = Eigen::MatrixXd(states_->states_.size() , states_->states_.size());   
        A(0,0) = 1; A(0,1) = 0; A(0,2) = -states_->getStates()["x_dot"]*sin(states_->getStates()["yaw"])*dt_.seconds(); A(0,3)= cos(states_->getStates()["yaw"])*dt_.seconds(); A(0,4) = 0;  
        A(1,0) = 0; A(1,1) = 1; A(1,2) =  states_->getStates()["x_dot"]*cos(states_->getStates()["yaw"])*dt_.seconds(); A(1,3)= sin(states_->getStates()["yaw"])*dt_.seconds(); A(1,4) = 0;  
        A(2,0) = 0; A(2,1) = 0; A(2,2) =  1                                                                                                        ; A(2,3)= 0                ; A(2,4) = dt_.seconds();  
        A(3,0) = 0; A(3,1) = 0; A(3,2) =  0                                                                                                        ; A(3,3)= 1                ; A(3,4) = 0;  
        A(4,0) = 0; A(4,1) = 0; A(4,2) =  0                                                                                                        ; A(4,3)= 0                ; A(4,4) = 1;  
        return A;
  
    }

    Eigen::MatrixXd ConstantHeadingRate::calcJacobianAndUpdate(const rclcpp::Time& current_time_)
    {

        dt_ = current_time_ - previous_time_;
        autodiff::VectorXreal state_(states_->states_.size());
        int index = 0;
        //creating autodiff variable from my state space for differentiation purpose
        for(auto it = states_->getStateOrder().begin() ; it!= states_->getStateOrder().end() ; ++it)
        {
            state_(index) = states_->getStates()[*it];
            // std::cout<<"S0:"<<state_(index)<<"\n";
            index++;
        }
        autodiff::VectorXreal newState;
        
        Eigen::MatrixXd J = jacobian( [&](auto state_){return this->update(state_); }, wrt(state_), at(state_), newState);
        // std::cout<<"states Jacobian:\n "<<J<<"\n";
        previous_time_ = current_time_;

        // updating state space 
        index = 0;
        for(auto it = states_->getStateOrder().begin() ; it!= states_->getStateOrder().end() ; ++it)
        {
            states_->getStates()[*it] = newState(index).val();
            //  std::cout<<"S1:"<<newState(index)<<"\n";
            index++;
        }
        normalizeAngle(states_->getStates()["yaw"]);


        return J;  //retrun nazari seg fault mide!!!!
    }

    void ConstantHeadingRate::setVelocity(const Filter::Velocity& velocity_)
    {

        states_->getStates()["x_dot"] = velocity_.x_dot;
        // state_["y_dot"] = velocity_.y_dot;
        // state_["z_dot"] = velocity_.z_dot;
        

    }

    void ConstantHeadingRate::setAngularVelocity(const Filter::AngularVelocity& angular_velocity_)
    {
        // state_["roll_dot"] = angular_velocity_.roll_dot;
        // state_["pitch_dot"] =  angular_velocity_.pitch_dot;
        states_->getStates()["yaw_dot"] = angular_velocity_.yaw_dot;

    }

    void ConstantHeadingRate::setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& twist_)
    {
        states_->getStates()["x_dot"] =  twist_.linear.x;
        states_->getStates()["yaw_dot"] = twist_.angular.z;
    }

    void ConstantHeadingRate::setStates(StateSpace* states_)
    {
        this->states_ = states_;
    }


    void ConstantHeadingRate::setPosition(const Position& position_)
    {

    }

    Filter::Position ConstantHeadingRate::getPosition()
    {
        Filter::Position position_; position_.x = states_->getStates()["x"] , position_.y = states_->getStates()["y"];
        return position_;
    }

    void ConstantHeadingRate::setAngle(const Angle& angle_)
    {
        
    }
    Filter::Angle ConstantHeadingRate::getAngle()
    {
        Filter::Angle angle_; angle_.yaw = states_->getStates()["yaw"];
        return angle_;
    }

    Filter::Velocity ConstantHeadingRate::getVelocity()
    {
        Filter::Velocity velocity_; velocity_.x_dot = states_->getStates()["x_dot"];
        return velocity_;
    }

    Filter::AngularVelocity ConstantHeadingRate::getAngularVelocity()
    {
        Filter::AngularVelocity angular_velocity_; angular_velocity_.yaw_dot = states_->getStates()["yaw_dot"];
        return angular_velocity_;
    }

    rclcpp::Duration ConstantHeadingRate::getDt()
    {
        return dt_;
    }

    void ConstantHeadingRate::normalizeAngle(double& angle_)
    {
        while(angle_>M_PI)
            angle_ -= 2.0*M_PI;
        while(angle_<-M_PI)
            angle_ += 2.0*M_PI;
    }
} //namespace Filter