#include "fusion/car.hpp"

namespace Filter{
    Car::Car()
    {
        std::cout<<"Ctor of Car \n";
    }

    void Car::init()
    {}

    void Car::update(const rclcpp::Time& current_time_)
    {}

    void Car::setVelocity(const Filter::Velocity& velocity_)
    {}

    void Car::setAngularVelocity(const Filter::AngularVelocity& angular_velocity_)
    {}

    void Car::setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& twist_)
    {}

} //namespace Filter