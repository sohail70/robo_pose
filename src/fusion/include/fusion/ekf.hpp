#ifndef EKF_HPP
#define EKF_HPP

#include<fusion/fusion.hpp>
#include<fusion/sensor_data.hpp>

namespace Filter{
    template<typename velType , typename imuType> 
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
            std::shared_ptr<VelocitySource<velType>>  velocity_source_;
            std::shared_ptr<ImuSource<imuType>> imu_source_;
        public:
            Ekf(std::unique_ptr<MotionModel>  , std::shared_ptr<VelocitySource<velType>> , std::shared_ptr<ImuSource<imuType>> );
            virtual void initialize() override;
            virtual void predict(const rclcpp::Time& ) override;
            virtual void update() override;
            virtual Eigen::MatrixXd getStates() override;
            virtual void setStates(StateSpace* ) override;

    };
} //namespace Filter
#endif