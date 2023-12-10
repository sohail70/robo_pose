#include "fusion/motion_model.hpp"
namespace Filter{
    MotionModel::MotionModel(): position_{0.0,0.0,0.0} ,angle_{0.0,0.0,0.0} , velocity_{0.0,0.0,0.0}, angular_velocity_{0.0,0.0,0.0}
    {
        std::cout<<"Ctor of MotionModel \n";
    }



} //namespace Filter