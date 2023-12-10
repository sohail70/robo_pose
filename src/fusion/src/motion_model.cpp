#include "fusion/motion_model.hpp"
namespace Filter{
    MotionModel::MotionModel(): position_{0.0,0.0,0.0} ,angle_{0.0,0.0,0.0} , velocity_{10.0,0.0,0.0}, angular_velocity_{0.0,0.0,5.0}
    {
        std::cout<<"Ctor of MotionModel \n";
    }

    MotionModel::~MotionModel(){}




} //namespace Filter