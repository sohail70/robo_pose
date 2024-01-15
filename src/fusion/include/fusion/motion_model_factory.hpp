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
                std::unique_ptr<MotionModel> createModelFromPlugin( std::string , std::shared_ptr<StateSpace> );
            private:
                using Creator = std::function<std::unique_ptr<MotionModel>()>;
                std::unordered_map<int , Creator> creators;
                std::unique_ptr<pluginlib::ClassLoader<Filter::MotionModel>> loader_;
        };
    
} // namespace Filter


#endif