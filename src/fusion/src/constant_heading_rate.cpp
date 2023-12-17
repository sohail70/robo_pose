
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
        // if(dt.seconds()>0.0001)
        // {
            position_.x = position_.x + velocity_.x_dot*cos(angle_.yaw)*dt_.seconds();
            position_.y = position_.y + velocity_.x_dot*sin(angle_.yaw)*dt_.seconds();
            angle_.yaw = angle_.yaw + angular_velocity_.yaw_dot*dt_.seconds();
            previous_time_ = current_time_;
        // }
    }

    void ConstantHeadingRate::setVelocity(const Filter::Velocity& velocity_)
    {
        this->velocity_.x_dot = velocity_.x_dot;
        this->velocity_.y_dot = velocity_.y_dot;
        this->velocity_.z_dot = velocity_.z_dot;
    }

    void ConstantHeadingRate::setAngularVelocity(const Filter::AngularVelocity& angular_velocity_)
    {
        this->angular_velocity_.pitch_dot = angular_velocity_.pitch_dot;
        this->angular_velocity_.roll_dot = angular_velocity_.roll_dot;
        this->angular_velocity_.yaw_dot = angular_velocity_.yaw_dot;
    }

    void ConstantHeadingRate::setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& twist_)
    {
        velocity_.x_dot = twist_.linear.x;
        velocity_.y_dot = twist_.linear.y;
        velocity_.z_dot = twist_.linear.z;
        angular_velocity_.roll_dot = twist_.angular.x;
        angular_velocity_.pitch_dot = twist_.angular.y;
        angular_velocity_.yaw_dot = twist_.angular.z;
    }

    Filter::Position& ConstantHeadingRate::getPosition()
    {
        return position_;
    }

    Filter::Angle& ConstantHeadingRate::getAngle()
    {
        return angle_;
    }

    Filter::Velocity& ConstantHeadingRate::getVelocity()
    {
        return velocity_;
    }

    Filter::AngularVelocity& ConstantHeadingRate::getAngularVelocity()
    {
        return angular_velocity_;
    }

    rclcpp::Duration ConstantHeadingRate::getDt()
    {
        return dt_;
    }
} //namespace Filter