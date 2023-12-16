#include<fusion/ekf.hpp>

namespace Filter{
    Ekf::Ekf(std::unique_ptr<MotionModel> motion_model_):Fusion(std::move(motion_model_))
    {
        std::cout<<"Ctor of Ekf \n";
    }

    void Ekf::predict(const rclcpp::Time& current_time_)
    {
        motion_model_->update(current_time_);
    }
} //namespace Filter