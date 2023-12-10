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

} //namespace Filter