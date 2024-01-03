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
            virtual void predict(const rclcpp::Time& ) override;
            virtual void update(const Observations& ) override;
            virtual void setMotionModel(std::unique_ptr<MotionModel> ) override;
            virtual Eigen::MatrixXd getStates() override;
            virtual void setStates(std::shared_ptr<StateSpace> ) override;

    };
} //namespace Filter
#endif