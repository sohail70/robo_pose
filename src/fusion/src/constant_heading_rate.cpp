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

#include "fusion/constant_heading_rate.hpp"
namespace Filter{
    ConstantHeadingRate::ConstantHeadingRate()
    {
        std::cout<<"Ctor of constant heading rate \n";
    }



    autodiff::VectorXreal ConstantHeadingRate::propagate(const autodiff::VectorXreal& state)
    {
        autodiff::VectorXreal newState(state.size());
        autodiff::real dt = autodiff::real(dt_.seconds()); 
        const auto index = states_->getStateOrder();
        // std::cout<<"x_dot:"<<(index.find("x") != index.end() )<<"\n";
        newState(0) = state(index.at("x")) + state(index.at("x_dot")) * cos(state(index.at("yaw"))) * dt; // Update x
        newState(1) = state(index.at("y")) + state(index.at("x_dot")) * sin(state(index.at("yaw"))) * dt; // Update y
        newState(2) = state(index.at("yaw")) + state(index.at("yaw_dot")) * dt;                 // Update yaw angle
        newState(3) = state(index.at("x_dot")) + state(index.at("x_ddot"))*dt;                                 
        newState(4) = state(index.at("yaw_dot"));                                
        newState(5) = state(index.at("x_ddot"));
        return newState;
    }


    // Eigen::MatrixXd ConstantHeadingRate::getJacobian() //Hardcoded jacobina for the [x,y,yaw,x_dot,yaw_dot,x_ddot] states
    // {
    //     Eigen::MatrixXd A = Eigen::MatrixXd(states_->states_.size() , states_->states_.size());   
    //     A(0,0) = 1; A(0,1) = 0; A(0,2) = -states_->getStates()[states_->getStateOrder().at("x_dot")].val()*sin(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(0,3)= cos(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(0,4) = 0; A(0,5) = 0; 
    //     A(1,0) = 0; A(1,1) = 1; A(1,2) =  states_->getStates()[states_->getStateOrder().at("x_dot")].val()*cos(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(1,3)= sin(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(1,4) = 0; A(1,5) = 0;
    //     A(2,0) = 0; A(2,1) = 0; A(2,2) =  1;A(2,3)= 0;A(2,4) = dt_.seconds();A(2,5) = 0;
    //     A(3,0) = 0; A(3,1) = 0; A(3,2) =  0;A(3,3)= 1;A(3,4) = 0; A(3,5) = dt_.seconds();
    //     A(4,0) = 0; A(4,1) = 0; A(4,2) =  0;A(4,3)= 0;A(4,4) = 1; A(4,5) = 0;
    //     A(5,0) = 0; A(5,1) = 0; A(5,2) =  0;A(5,3)= 0;A(5,4) = 0; A(5,5) = 1;

    //     return A;
  
    // }

} //namespace Filter