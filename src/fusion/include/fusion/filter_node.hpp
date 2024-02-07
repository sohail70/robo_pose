// MIT License
//
// Copyright (c) 2023 Soheil Espahbodi Nia
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
#include<nav_msgs/msg/odometry.hpp>
#include<std_msgs/msg/float64.hpp>
#include<geometry_msgs/msg/transform_stamped.hpp>
#include<tf2_ros/transform_listener.h>
#include<tf2_ros/buffer.h>
#include<tf2_ros/transform_broadcaster.h>

namespace Filter{
    struct ThreadParams{
        std::shared_ptr<Visualization::Visualization> visualization_;
        rclcpp::Time time_;
    };

    class FilterNode: public rclcpp::Node{
        public:
            FilterNode(rclcpp::NodeOptions );
            ~FilterNode();
            void initialize();
            std::shared_ptr<StateSpace> getStateSpace();

        private:
            std::shared_ptr<StateSpace> states_;
            std::unique_ptr<MotionModelFactory> model_factory_;
            std::unique_ptr<MotionModel> model_;
            std::unique_ptr<FilterFactory> filter_factory_;
            std::unique_ptr<Fusion> filter_;

            std::unordered_map<std::string , std::vector<std::string>> sensor_states_;
            std::vector<rclcpp::SubscriptionBase::SharedPtr> sensor_subs_;

            rclcpp::TimerBase::SharedPtr timer_;
            void imuCallback(const sensor_msgs::msg::Imu::SharedPtr  , std::string );
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr , std::string);
            void controlCallback(const geometry_msgs::msg::Twist::SharedPtr );
            void timerCallback();

            std::priority_queue<Observations>  observations_; 
            MotionModel* local_motion_model_;

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
            nav_msgs::msg::Odometry filtered_odom_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_odom_pub_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_filter_pub_;


            rclcpp::Time previous_update_time_;

            std::shared_ptr<Visualization::Visualization> visualization_;
            ThreadParams params_;
            std::thread rviz_marker_;
            void rviz_marker(ThreadParams* params_) {
                rclcpp::Rate loop_rate(200);
                // geometry_msgs::msg::Pose2D pose_;
                geometry_msgs::msg::Pose pose_;
                while (rclcpp::ok())
                {
                    autodiff::VectorXreal sta_ = filter_->getStates();
                    auto index_ = states_->getStateOrder();

                    pose_.position.x = sta_(index_.at("x")).val();
                    pose_.position.y = sta_(index_.at("y")).val();
                    pose_.position.z = sta_(index_.at("z")).val();
                    
                    tf2::Quaternion quaternion_;
                    quaternion_.setRPY(index_.count("roll") ? sta_(index_.at("roll")).val() : 0,
                                       index_.count("pitch") ? sta_(index_.at("pitch")).val() : 0,
                                       index_.count("yaw") ? sta_(index_.at("yaw")).val() : 0);
                    geometry_msgs::msg::Quaternion orientation_msg_;
                    tf2::convert(quaternion_, orientation_msg_);
                    pose_.orientation = orientation_msg_;


                    auto t_ = params_->time_;
                    params_->visualization_->clear();
                    params_->visualization_->addArrow(pose_);
                    params_->visualization_->publishArrow(t_);
                    loop_rate.sleep();
                }
            }

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            geometry_msgs::msg::TransformStamped odom_base_link_transform_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> odom_base_link_broadcaster_;


            std::unordered_map<std::string, std::function<void(const sensor_msgs::msg::Imu::SharedPtr,
                                                               Observations &,
                                                               std::unordered_map<std::string, int>)>> imu_state_action_;
            std::unordered_map<std::string, std::function<void(const nav_msgs::msg::Odometry::SharedPtr , 
                                                               Observations &,
                                                               std::unordered_map<std::string, int>)>> odom_state_action_;
            void initializeStateAction();

            std::string odom_frame_;
            std::string base_link_frame_;
            bool publish_tf_;
    };
} // namespace Filter

#endif