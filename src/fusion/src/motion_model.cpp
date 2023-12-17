#include "fusion/motion_model.hpp"
namespace Filter{
    MotionModel::MotionModel(): position_{0.0,0.0,0.0} ,angle_{0.0,0.0,0.0} , velocity_{0.0,0.0,0.0}, angular_velocity_{0.0,0.0,0.0}, dt_(rclcpp::Duration::from_seconds(0))
    {
        std::cout<<"Ctor of MotionModel \n";
        
    }

    MotionModel::~MotionModel(){}




} //namespace Filter