#ifndef FILTER_NODE_HPP
#define FILTER_NODE_HPP
#include<rclcpp/rclcpp.hpp>
#include<autodiff/forward/real.hpp>
#include<autodiff/forward/real/eigen.hpp>
#include<fusion/state_space.hpp>
#include<fusion/ekf.hpp>
#include<fusion/sensor_data.hpp>
#include<fusion/motion_model.hpp>
#include<fusion/motion_model_factory.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include<fusion/visualization.hpp>
#include<fusion/mediator.hpp>
#include<fusion/filter_factory.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<fusion/observations.hpp>
#include<fusion/visualization.hpp>
#include<queue>
namespace Filter{
    class FilterNode: public rclcpp::Node{
        public:
            FilterNode(rclcpp::NodeOptions );
            void initialize();
            std::shared_ptr<StateSpace> getStateSpace();

        private:
            std::shared_ptr<StateSpace> states_;
            std::unique_ptr<MotionModelFactory> model_factory_;
            std::unique_ptr<MotionModel> model_;
            // std::unique_ptr<MessageHub> hub_;
            std::unique_ptr<FilterFactory> filter_factory_;
            std::unique_ptr<Fusion> filter_;
            std::shared_ptr<Visualization::Visualization> visualization_;

            std::unordered_map<std::string , std::vector<std::string>> sensor_states_;
            std::vector<rclcpp::SubscriptionBase::SharedPtr> sensor_subs_;

            rclcpp::TimerBase::SharedPtr timer_;
            void imuCallback(const sensor_msgs::msg::Imu::SharedPtr  , std::string );
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr , std::string);
            void controlCallback(const geometry_msgs::msg::Twist::SharedPtr );
            void timerCallback();

            std::priority_queue<Observations>  observations_; 
            MotionModel* local_motion_model_;



    };
} // namespace Filter


#endif