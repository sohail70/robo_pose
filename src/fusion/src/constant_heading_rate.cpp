
#include "fusion/constant_heading_rate.hpp"

namespace Filter{
    ConstantHeadingRate::ConstantHeadingRate()
    {
        std::cout<<"Ctor of constant heading rate \n";
        init();
    }

    void ConstantHeadingRate::init()
    {


    }

    void ConstantHeadingRate::update(const rclcpp::Time& current_time_)
    {
        rclcpp::Duration dt = current_time_ - previous_time_;
        position_.x = position_.x + velocity_.x_dot*cos(angular_velocity_.yaw_dot)*dt.seconds();
        position_.y = position_.y + velocity_.x_dot*sin(angular_velocity_.yaw_dot)*dt.seconds();
        angle_.yaw = angle_.yaw + angular_velocity_.yaw_dot*dt.seconds();
        previous_time_ = current_time_;
    }
    

} //namespace Filter