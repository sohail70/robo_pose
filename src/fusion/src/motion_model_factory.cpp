#include "fusion/motion_model_factory.hpp"


namespace Filter{
    MotionModelFactory::MotionModelFactory()
    {
       creators[static_cast<int>(ModelType::CONSTANT_HEADING_RATE)] = [](){return std::make_unique<ConstantHeadingRate>();};
       creators[static_cast<int>(ModelType::CAR)] = [](){return std::make_unique<Car>();};
    }

    std::unique_ptr<MotionModel> MotionModelFactory::createModel(ModelType type_ , std::shared_ptr<StateSpace> states_)
    {
        std::unordered_map<int, Creator>::iterator it_ = creators.find(static_cast<int>(type_));

        if(it_ != creators.end())
        {
            auto model_ = it_->second();
            model_->setStates(states_);
            return model_;
        }
        return nullptr;

    }

    std::unique_ptr<MotionModel> MotionModelFactory::createModelFromPlugin(std::string plugin_class_name_, std::shared_ptr<StateSpace> states_)
    {
        pluginlib::ClassLoader<Filter::MotionModel> loader("fusion", "Filter::MotionModel");
        std::unique_ptr<Filter::MotionModel> model_;
        try
        {
            auto load = loader.createUniqueInstance(plugin_class_name_);
            model_.reset(load.release());
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
        model_->setStates(states_);
        return model_;
    }
} //namespace Filter