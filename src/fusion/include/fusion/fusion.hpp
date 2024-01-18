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

#ifndef FUSION_HPP
#define FUSION_HPP

#include<iostream>
#include<memory>
#include<fusion/motion_model.hpp>
#include<rclcpp/rclcpp.hpp>
#include<Eigen/Dense>
#include<fusion/state_space.hpp>
#include<fusion/measurement_model.hpp>
#include<fusion/observations.hpp>
#include<tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/quaternion.hpp>

namespace Filter{
    class Fusion{
        protected:
            std::unique_ptr<MotionModel> motion_model_;
            std::shared_ptr<StateSpace> states_;
            // std::unique_ptr<MeasurementModel> measurement_model_;
        public:
            Fusion();
            Fusion(std::unique_ptr<MotionModel> );
            virtual void initialize() = 0;
            virtual void predict(const rclcpp::Time& , const rclcpp::Duration&) = 0;
            virtual void update(const Observations&) = 0;
            virtual void setMotionModel(std::unique_ptr<MotionModel> ) = 0;
            virtual autodiff::VectorXreal getStates() = 0;
            virtual void setStates(std::shared_ptr<StateSpace> ) = 0;
            virtual void setProcessNoise(std::vector<double> ) = 0;
            virtual void setMeasurementNoise(std::vector<double> ) = 0;
    };

} //namespace Filter
#endif