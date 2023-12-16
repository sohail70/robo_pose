#include<fusion/fusion.hpp>


namespace Filter{
    class Ekf: public Fusion{
        private:
            Eigen::MatrixXd P; //Covariance matrix
            Eigen::MatrixXd A; //Linearized model
            Eigen::MatrixXd B; //Control Matrix
            Eigen::MatrixXd Q; //Process Noise Matrix
            Eigen::MatrixXd H; //Measuring Matrix 
            Eigen::MatrixXd R; //Measurment Noise covariance Matrix
            Eigen::MatrixXd I; //Identiy Matrix
        public:
            Ekf(std::unique_ptr<MotionModel> );
            virtual void initialize() override;
            virtual void predict(const rclcpp::Time& ) override;
            virtual void update() override;
    };
} //namespace Filter