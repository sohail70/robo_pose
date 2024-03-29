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

#include "fusion/motion_model.hpp"
namespace Filter{
    MotionModel::MotionModel(): dt_(rclcpp::Duration::from_seconds(0))
    {
        std::cout<<"Ctor of MotionModel \n";
        
    }

    MotionModel::~MotionModel(){}

    autodiff::MatrixXreal MotionModel::update(const rclcpp::Time& current_time_ , const rclcpp::Duration& dt)
    {
        static rclcpp::Time previous_time_ = current_time_;
        dt_ = dt;
        autodiff::VectorXreal& state_ = states_->getStates();
        autodiff::VectorXreal newState; 
        
        //////Test///////////        
        // std::cout<<"states Jacobian Test:\n "<<this->getJacobian()<<"\n \n";
        ////////////////////
        autodiff::MatrixXreal J = jacobian( [&](auto state_){
            return this->propagate(state_); 
            }, wrt(state_), at(state_), newState);
        previous_time_ = current_time_;
        
        for(int i = 0 ; i<state_.size() ; i++)
            state_(i) = newState(i).val();
        normalizeAngle(states_->getStates()[states_->getStateOrder().at("yaw")].val());
        return J; 
    }

    void MotionModel::setStates(std::shared_ptr<StateSpace> states_)
    {
        this->states_ = states_;
    }

    void MotionModel::setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& twist_)
    {
        if(twist_.linear.x == 0)
            states_->getStates()[states_->getStateOrder().at("x_dot")] = twist_.linear.x;
    }
    void MotionModel::normalizeAngle(double& angle_)
    {
        while(angle_>M_PI)
            angle_ -= 2.0*M_PI;
        while(angle_<-M_PI)
            angle_ += 2.0*M_PI;
    }
} //namespace Filter