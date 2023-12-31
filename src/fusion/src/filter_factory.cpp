#include<fusion/filter_factory.hpp>

namespace Filter{
    FilterFactory::FilterFactory(){
        filters_[static_cast<int>(FilterType::EKF)] = [](){return std::make_unique<Ekf>();};
    }

    std::unique_ptr<Fusion> FilterFactory::createFilter(FilterType type_ ,  std::unique_ptr<MotionModel> motion_model_)
    {
       std::unordered_map<int ,Creator>::iterator  it_ = filters_.find(static_cast<int>(type_));
       if(it_ != filters_.end())
       {
            it_->second()->setMotionModel(std::move(motion_model_));
            return it_->second();
       }
       return nullptr;
    }
} // namespace Filter