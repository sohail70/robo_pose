
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

    void ConstantHeadingRate::update(const rclcpp::Time& current_time_)
    {
        dt_ = current_time_ - previous_time_;

            // position_.x = position_.x + velocity_.x_dot*cos(angle_.yaw)*dt_.seconds();
            // position_.y = position_.y + velocity_.x_dot*sin(angle_.yaw)*dt_.seconds();
            // angle_.yaw = angle_.yaw + angular_velocity_.yaw_dot*dt_.seconds();
            // normalizeAngle(angle_.yaw);
            // previous_time_ = current_time_;
        auto& state_ = states_->getStates(); //don't forget the & because we don't wanna update a copy of the states!
        state_["x"] += state_["x_dot"]*cos(state_["yaw"])*dt_.seconds();
        state_["y"] += state_["x_dot"]*sin(state_["yaw"])*dt_.seconds();
        state_["yaw"] += state_["yaw_dot"]*dt_.seconds();
        normalizeAngle(state_["yaw"]);
        previous_time_ = current_time_;


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