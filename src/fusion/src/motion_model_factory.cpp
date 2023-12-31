#include "fusion/motion_model_factory.hpp"


namespace Filter{
    MotionModelFactory::MotionModelFactory()
    {
       creators[static_cast<int>(ModelType::CONSTANT_HEADING_RATE)] = [](){return std::make_unique<ConstantHeadingRate>();};
       creators[static_cast<int>(ModelType::CAR)] = [](){return std::make_unique<Car>();};
    }

    std::unique_ptr<MotionModel> MotionModelFactory::createModel(ModelType type_ , StateSpace* states_)
    {
        std::unordered_map<int, Creator>::iterator it_ = creators.find(static_cast<int>(type_));

        if(it_ != creators.end())
        {
            it_->second()->setStates(states_);
            return it_->second();
        }
        return nullptr;

    }
} //namespace Filter