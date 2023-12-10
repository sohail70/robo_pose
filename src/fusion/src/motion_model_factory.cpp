#include "fusion/motion_model_factory.hpp"


namespace Filter{
    MotionModelFactory::MotionModelFactory()
    {
       creators[static_cast<int>(ModelType::CONSTANT_HEADING_RATE)] = [](){return std::make_unique<ConstantHeadingRate>();};
       creators[static_cast<int>(ModelType::CAR)] = [](){return std::make_unique<Car>();};
    }

    std::unique_ptr<MotionModel> MotionModelFactory::createModel(ModelType type)
    {
        std::unordered_map<int, Creator>::iterator it = creators.find(static_cast<int>(type));

        if(it != creators.end())
        {
            return it->second();
        }
        return nullptr;

    }
} //namespace Filter