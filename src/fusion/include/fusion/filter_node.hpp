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
#include<queue>
namespace Filter{
    class FilterNode: public rclcpp::Node{
        public:
            FilterNode(rclcpp::NodeOptions );
            void initialize();
            std::unique_ptr<StateSpace> getStateSpace();

        private:
            std::unique_ptr<StateSpace> states_;
            std::unique_ptr<MotionModelFactory> model_factory_;
            std::unique_ptr<MotionModel> model_;
            // std::unique_ptr<MessageHub> hub_;
            std::unique_ptr<FilterFactory> filter_factory_;
            std::unique_ptr<Fusion> filter_;

            std::unordered_map<std::string , std::vector<std::string>> sensor_states_;
            std::vector<rclcpp::SubscriptionBase::SharedPtr> sensor_subs_;
            // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_callback_;
            // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_callback_;
            // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_callback_;

            void imuCallback(const sensor_msgs::msg::Imu::SharedPtr  , std::string );
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr , std::string);

            struct Observations{
                rclcpp::Time time_;
                autodiff::VectorXreal states_;
                bool operator<(const Observations& other) const{
                    return time_<other.time_; //lower time has priority
                }
            };
            std::priority_queue<Observations>  observations_; 



    };
} // namespace Filter


#endif