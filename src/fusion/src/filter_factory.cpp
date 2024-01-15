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

#include<fusion/filter_factory.hpp>

namespace Filter{
    FilterFactory::FilterFactory(){
        filters_[static_cast<int>(FilterType::EKF)] = [](){return std::make_unique<Ekf>();};
    }

    std::unique_ptr<Fusion> FilterFactory::createFilter(FilterType type_ ,  std::unique_ptr<MotionModel> motion_model_ , std::shared_ptr<StateSpace> states_)
    {
       std::unordered_map<int ,Creator>::iterator  it_ = filters_.find(static_cast<int>(type_));
       if(it_ != filters_.end())
       {
            auto filter_ = it_->second(); //creates a new object -- mind you second and second() are very different!  second() invokes the lambda and it gives you the filter but second gives you the function pointer it self! thats why each second() gives you a new object!
            filter_->setStates(states_);
            filter_->setMotionModel(std::move(motion_model_));
            return filter_;
       }
       return nullptr;
    }
} // namespace Filter