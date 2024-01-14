#ifndef MOTION_MODEL_FACTORY_HPP
#define MOTION_MODEL_FACTORY_HPP


#include"fusion/motion_model.hpp"
#include"fusion/constant_heading_rate.hpp"
#include"fusion/car.hpp"
#include<functional>
#include<unordered_map>
#include<memory>
#include<pluginlib/class_loader.hpp>
namespace Filter
{
    enum class ModelType{CONSTANT_HEADING_RATE , CAR};
    class MotionModelFactory{
            public:
                MotionModelFactory();
                std::unique_ptr<MotionModel> createModel( ModelType , std::shared_ptr<StateSpace> );
                std::shared_ptr<MotionModel> createModelFromPlugin( std::string , std::shared_ptr<StateSpace> );
            private:
                using Creator = std::function<std::unique_ptr<MotionModel>()>;
                std::unordered_map<int , Creator> creators;
        };
    
} // namespace Filter


#endif