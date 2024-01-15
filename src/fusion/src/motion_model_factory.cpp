// MIT License
//
// Copyright (c) 2023 Soheil Espahbodi Nia
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
        loader_ = std::make_unique<pluginlib::ClassLoader<Filter::MotionModel>>("fusion", "Filter::MotionModel");
        std::unique_ptr<Filter::MotionModel> model_;
        try
        {
            auto load_ = loader_->createUniqueInstance(plugin_class_name_);
            model_.reset(load_.release());
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
        model_->setStates(states_);
        return model_;
    }
} //namespace Filter