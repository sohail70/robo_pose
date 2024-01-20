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

#include<fusion/ekf.hpp>
#include<fusion/constant_heading_rate.hpp>

namespace Filter{

    Ekf::Ekf()
    {
        std::cout<<"Ctor of Ekf \n";
    }
    
    Ekf::Ekf(std::unique_ptr<MotionModel> motion_model_):
    Fusion(std::move(motion_model_)) 
    {
        std::cout<<"Ctor of Ekf \n";
    }

    void Ekf::setStates(std::shared_ptr<StateSpace> states_)
    {
        this->states_ = states_;
    }

    void Ekf::setMotionModel(std::unique_ptr<MotionModel> motion_model_)
    {
        this->motion_model_ = motion_model_ ? std::move(motion_model_) : nullptr;
    }

    void Ekf::setProcessNoise(std::vector<double> Q_)
    {
        int num_states_ = states_->getStates().size();
        int counter_ = 0;
        Q.setZero(num_states_ , num_states_);
        for(int i = 0 ; i <num_states_ ; i++)
        {
            for(int j = 0 ; j<num_states_ ; j++)
            {
                Q(i,j) = Q_.at(counter_);
                counter_++;
            }
        }
    }

    void Ekf::setMeasurementNoise(std::vector<double> R_)
    {
        int num_states_ = states_->getStates().size();
        int counter_ = 0;
        R.setZero(num_states_ , num_states_);
        for(int i = 0 ; i <num_states_ ; i++)
        {
            for(int j = 0 ; j<num_states_ ; j++)
            {
                R(i,j) = R_.at(counter_);
                counter_++;
            }
        }
    }

    void Ekf::initialize()
    {
        int num_states_ = states_->getStates().size(); //[x,y,yaw,x_dot,yaw_dot,x_ddot]
        P.setZero(num_states_,num_states_);
        I = Eigen::MatrixXd::Identity(num_states_ , num_states_);
    }

    void Ekf::predict(const rclcpp::Time& current_time_ , const rclcpp::Duration& dt_)
    {
        A = motion_model_->update(current_time_ , dt_);
        P = A*P*A.transpose() + Q;
        // std::cout<<"P0: "<<P<<"\n \n \n";
    }

    void Ekf::update(const Observations& observation_)
    {
        autodiff::VectorXreal& X = states_->getStates();
        auto H = observation_.H;
        auto real_measurement_ = observation_.states_;
        autodiff::MatrixXreal measurement_prediction_ = H*X;
        autodiff::MatrixXreal measurement_residual_ = real_measurement_ - measurement_prediction_;
        autodiff::MatrixXreal innovation_covariance_ = H*P*H.transpose() + R;
        autodiff::MatrixXreal kalman_gain_ = P*H.transpose()*innovation_covariance_.completeOrthogonalDecomposition().pseudoInverse(); //.inverse() doesn't work because we have 0 in the diagonal due to my design choice of having the full states in the H matrix!
        // autodiff::MatrixXreal kalman_gain_ = P*H.transpose()*innovation_covariance_.inverse();
        X = X + kalman_gain_*measurement_residual_;
        P = (I-kalman_gain_*H)*P*(I-kalman_gain_*H).transpose() + kalman_gain_*R*kalman_gain_.transpose(); // or use P = (I-K*H)*P
        // P = (I-kalman_gain_*H)*P;
        // std::cout<<"P: "<<P<<"\n \n \n ";
        std::cout<<"X: \n"<<X<<"\n \n \n ";
        std::cout<<"---------------- \n";
    }

    autodiff::VectorXreal Ekf::getStates()
    {
        return states_->getStates();
    }



} //namespace Filter