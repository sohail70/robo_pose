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

#ifndef MEDIATOR_HPP
#define MEDIATOR_HPP
#include<iostream>
#include<fusion/motion_model.hpp>
#include<geometry_msgs/msg/twist.hpp>
namespace Filter{

    struct Message{
        enum class MessageType{
            VELOCITY_UPDATE,
        };

        MessageType type;
        geometry_msgs::msg::Twist velocityData;        
    };

    class Mediator{
        public:
            virtual void notifyMessage(const Message& ) = 0;
    };

    class MessageHub: public Mediator{
        public:
        MessageHub(MotionModel* );
            virtual void notifyMessage(const Message& ) override;

        private:
            MotionModel* motion_model_;
    };

}; // namespace Filter





#endif