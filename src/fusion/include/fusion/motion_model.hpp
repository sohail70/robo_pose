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

#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include<iostream>
// #include<Eigen/Dense>
#include<cmath>
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<fusion/state_space.hpp>
#include<autodiff/forward/real.hpp>
#include<autodiff/forward/real/eigen.hpp>

namespace Filter{
    struct Position{
        double x, y, z;
    };
    struct Angle{
        double roll, pitch, yaw;
    };
    struct Velocity{
        double x_dot,y_dot,z_dot;
    };
    struct AngularVelocity{
        double roll_dot, pitch_dot , yaw_dot;
    };
    class MotionModel{
            protected:
                Position position_;
                Angle angle_;
                Velocity velocity_;
                AngularVelocity angular_velocity_;
                rclcpp::Duration dt_;
                rclcpp::Time previous_time_;
                std::shared_ptr<StateSpace> states_;
            public:
                MotionModel();
                virtual ~MotionModel();
                virtual autodiff::VectorXreal propagate(const autodiff::VectorXreal&)=0;
                // virtual Eigen::MatrixXd getJacobian() = 0;

                autodiff::MatrixXreal update(const rclcpp::Time&  , const rclcpp::Duration&);
                void setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& );
                void setStates(std::shared_ptr<StateSpace> states_);
                void normalizeAngle(double& );

        };
} //namespace Filter
#endif