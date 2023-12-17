#ifndef FUSION_HPP
#define FUSION_HPP

#include<iostream>
#include<memory>
#include<fusion/motion_model.hpp>
#include<rclcpp/rclcpp.hpp>
#include<Eigen/Dense>

namespace Filter{
    class Fusion{
        protected:
            std::unique_ptr<MotionModel> motion_model_;
            // std::unique_ptr<MeasurementModel> measurement_model_;
        public:
            Fusion(std::unique_ptr<MotionModel> );
            virtual void initialize() = 0;
            virtual void predict(const rclcpp::Time&) = 0;
            virtual void update() = 0;

    };

} //namespace Filter
#endif