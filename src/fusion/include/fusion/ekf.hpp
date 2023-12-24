#ifndef EKF_HPP
#define EKF_HPP

#include<fusion/fusion.hpp>
#include<fusion/sensor_data.hpp>

namespace Filter{
    template<typename velType , typename imuType> 
    class Ekf: public Fusion{
        private:
            Eigen::MatrixXd P; //Covariance matrix
            Eigen::MatrixXd A; //Linearized model
            Eigen::MatrixXd B; //Control Matrix
            Eigen::MatrixXd Q; //Process Noise Matrix
            Eigen::MatrixXd H; //Measuring Matrix 
            Eigen::MatrixXd R; //Measurment Noise covariance Matrix
            Eigen::MatrixXd I; //Identiy Matrix
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