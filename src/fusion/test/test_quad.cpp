#include <gtest/gtest.h>
#include <fusion/state_space.hpp>
#include <fusion/motion_model_factory.hpp>
#include <fusion/quadcopter.hpp>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <fusion/visualization.hpp>
#include<tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>


void normalizeAngle(double& angle_)
    {
        while(angle_>M_PI)
            angle_ -= 2.0*M_PI;
        while(angle_<-M_PI)
            angle_ += 2.0*M_PI;
    }

TEST(MotionModel, quadcopter)
{
    rclcpp::init(0,nullptr);
    std::vector<std::string> config_states_ = std::vector<std::string>{"x", "y", "z",
                                                                       "roll", "pitch", "yaw",
                                                                       "x_dot", "y_dot", "z_dot",
                                                                       "roll_dot", "pitch_dot", "yaw_dot",
                                                                       "x_ddot", "y_ddot", "z_ddot"};
    auto states_ = std::make_shared<Filter::StateSpace>(config_states_);
    states_->updateStates({0.0,0.0,0.0,
                           0.0,0.0,0.0,
                           0.0,0.0,0.01,
                           0.0,0.0,0.05,
                           0.0,0.0,0.0,
                         });
    auto model_factory_ = std::make_unique<Filter::MotionModelFactory>();
    auto model_ = model_factory_->createModel( Filter::ModelType::QUADCOPTER,  states_);
    rclcpp::Time current_time_;
    rclcpp::Time previous_time_ = rclcpp::Clock().now();
    rclcpp::Rate loop_rate_(2);
    auto index_ = states_->getStateOrder();
    Visualization::Visualization visualization_;
    geometry_msgs::msg::Pose pose_;
    while(rclcpp::ok()) {
        current_time_ = rclcpp::Clock().now();
        autodiff::VectorXreal& state_ = states_->getStates();
        auto dt_ = current_time_ - previous_time_;
        model_->update(current_time_,dt_);
        for(int i = 0 ; i<state_.size() ; i++)
            state_(i) = state_(i).val();
        normalizeAngle(states_->getStates()[states_->getStateOrder().at("yaw")].val());

        // Visualization
        pose_.position.x = state_(index_.at("x")).val();
        pose_.position.y = state_(index_.at("y")).val();
        pose_.position.z = state_(index_.at("z")).val();
        tf2::Quaternion quaternion_;
        quaternion_.setRPY(index_.count("roll") ? state_(index_.at("roll")).val() : 0,
                           index_.count("pitch") ? state_(index_.at("pitch")).val() : 0,
                           index_.count("yaw") ? state_(index_.at("yaw")).val() : 0);
        geometry_msgs::msg::Quaternion orientation_msg_;
        tf2::convert(quaternion_, orientation_msg_);
        pose_.orientation = orientation_msg_;



        visualization_.clear();
        visualization_.addArrow(pose_);
        visualization_.publishArrow(current_time_);
        loop_rate_.sleep();
    }

}