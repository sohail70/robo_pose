#include<iostream>
#include<memory>
#include<fusion/motion_model.hpp>
#include<rclcpp/rclcpp.hpp>
namespace Filter{
    class Fusion{
        protected:
            std::unique_ptr<MotionModel> motion_model_;
            // std::unique_ptr<MeasurementModel> measurement_model_;
        public:
            Fusion(std::unique_ptr<MotionModel> );
            virtual void predict(const rclcpp::Time&) = 0;
            virtual void update() = 0;
    };

} //namespace Filter