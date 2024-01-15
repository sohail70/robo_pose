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

#ifndef EKF_HPP
#define EKF_HPP

#include<fusion/fusion.hpp>
#include<fusion/sensor_data.hpp>

namespace Filter{
    class Ekf: public Fusion{
        private:
            autodiff::MatrixXreal P; //Covariance matrix
            autodiff::MatrixXreal A; //Linearized model
            autodiff::MatrixXreal B; //Control Matrix
            autodiff::MatrixXreal Q; //Process Noise Matrix
            autodiff::MatrixXreal H; //Measuring Matrix 
            autodiff::MatrixXreal R; //Measurment Noise covariance Matrix
            autodiff::MatrixXreal I; //Identiy Matrix
            // Eigen::MatrixXd X; //State Space
           
        public:
            Ekf();
            Ekf(std::unique_ptr<MotionModel> );
            virtual void initialize() override;
            virtual void predict(const rclcpp::Time&  , const rclcpp::Duration&) override;
            virtual void update(const Observations& ) override;
            virtual void setMotionModel(std::unique_ptr<MotionModel> ) override;
            virtual autodiff::VectorXreal getStates() override;
            virtual void setStates(std::shared_ptr<StateSpace> ) override;

    };
} //namespace Filter
#endif