#include<fusion/fusion.hpp>

namespace Filter{
    Fusion::Fusion(std::unique_ptr<MotionModel> motion_model_): motion_model_( motion_model_ ? std::move(motion_model_) : nullptr)
    {
        std::cout<<"Ctor of Fusion \n";
    }
} //namespace Filter