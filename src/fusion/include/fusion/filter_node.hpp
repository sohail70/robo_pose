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

namespace Filter{
    struct ThreadParams{
        // rclcpp::Node::SharedPtr node_;
        std::shared_ptr<Visualization::Visualization> visualization_;
        rclcpp::Time time_;
        // std::mutex visualization_mutex_;
        // std::condition_variable cv_;
        // bool ready;
    };

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
            // rclcpp::Duration dt_;

            std::shared_ptr<Visualization::Visualization> visualization_;
            ThreadParams params_;
            std::thread rviz_marker_;
            void rviz_marker(ThreadParams* params_) {
                rclcpp::Rate loop_rate(200);
                geometry_msgs::msg::Pose2D pose_;
                while (rclcpp::ok())
                {
                    autodiff::VectorXreal states_ = filter_->getStates();
                    pose_.x = states_(0).val();
                    pose_.y = states_(1).val();
                    pose_.theta = states_(2).val();
                    // RCLCPP_INFO(this->get_logger() , "x,y,theta: %f , %f, %f" , pose_.x , pose_.y , pose_.theta);
                    // RCLCPP_INFO(this->get_logger() , "state theta 1: %f" , pose_.theta);
                    auto t_ = params_->time_;
                    params_->visualization_->addArrow(pose_);
                    params_->visualization_->publishArrow(t_);
                    params_->visualization_->initialize();
                    loop_rate.sleep();
                }
            }

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            std::unordered_map<std::string, std::function<void(const sensor_msgs::msg::Imu::SharedPtr,
                                                               Observations &,
                                                               std::unordered_map<std::string, int>)>> imu_state_action_;
            void initializeStateAction();
    };
} // namespace Filter

#endif