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
            std::unique_ptr<MessageHub> hub_;
            std::unique_ptr<FilterFactory> filter_factory_;
            std::unique_ptr<Fusion> filter_;
            

    };
} // namespace Filter


#endif