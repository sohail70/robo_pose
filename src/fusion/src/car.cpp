#include "fusion/car.hpp"

namespace Filter{
    Car::Car()
    {
        std::cout<<"Ctor of Car \n";
    }

    void Car::init()
    {}

    autodiff::VectorXreal Car::update(const autodiff::VectorXreal&)
    {}

    void Car::setVelocity(const Filter::Velocity& velocity_)
    {}

    void Car::setAngularVelocity(const Filter::AngularVelocity& angular_velocity_)
    {}

    void Car::setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& twist_)
    {}

    void Car::setStates(StateSpace* states_)
    {}

    Eigen::MatrixXd Car::getJacobian()
    {}

    Eigen::MatrixXd Car::calcJacobianAndUpdate(const rclcpp::Time& ) 
    {}

    rclcpp::Duration Car::getDt()
    {}
} //namespace Filter