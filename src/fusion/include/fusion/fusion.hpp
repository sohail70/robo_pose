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
            virtual void predict(const rclcpp::Time&) = 0;
            virtual void update(const Observations&) = 0;
            virtual void setMotionModel(std::unique_ptr<MotionModel> ) = 0;
            virtual autodiff::VectorXreal getStates() = 0;
            virtual void setStates(std::shared_ptr<StateSpace> ) = 0;


    };

} //namespace Filter
#endif