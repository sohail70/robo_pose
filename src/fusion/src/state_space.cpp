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

#include<fusion/state_space.hpp>
#include<rclcpp/rclcpp.hpp>
namespace Filter{
    StateSpace::StateSpace(std::vector<std::string> state_names_)
    {
        states_.resize(state_names_.size());
        state_names_.resize(state_names_.size());
        std::cout<<"Ctor of state space \n";
        int index = 0;
        for(auto& state_name_ :state_names_)
        {
            states_[index] = 0.0;
            states_name_[state_name_] = index;
            index++;
        }

    }

    void StateSpace::updateStates(std::vector<double> values_)
    {
        int index = 0;
        for(auto&  value_ : values_)
        {
            states_[index] = values_[index];
            index++;
        }
    }
    // TODO: This should be const ref but for now im using it to update the states_ in the constant heading rate function and ekf so it'd be best to use updatesStates to update things!
    autodiff::VectorXreal& StateSpace::getStates()
    {
        return states_;
    }

    const std::unordered_map<std::string,int>& StateSpace::getStateOrder() {
        return states_name_;
    }

};