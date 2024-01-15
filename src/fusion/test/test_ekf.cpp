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

#include<gtest/gtest.h>
#include<fusion/ekf.hpp>
#include<fusion/sensor_data.hpp>
#include<fusion/motion_model.hpp>
#include<fusion/motion_model_factory.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include<fusion/visualization.hpp>
#include<fusion/mediator.hpp>
TEST(Ekf , ekf)
{
    ASSERT_EQ(1,1);
    rclcpp::init(0,nullptr);
    std::shared_ptr<VelocitySource<geometry_msgs::msg::Twist>> velocity_source_ = std::make_shared<RosCmdVelSource>();
    std::shared_ptr<ImuSource<sensor_msgs::msg::Imu>> imu_source_ = std::make_shared<RosImuSource>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node( dynamic_cast<RosCmdVelSource*>(velocity_source_.get())->getRosNode());
    executor->add_node(dynamic_cast<RosImuSource*>(imu_source_.get())->getRosNode());


    std::vector<std::string> state_names_{"x" , "y" , "yaw" , "x_dot" , "yaw_dot"};
    Filter::StateSpace states_(state_names_);
    Filter::MotionModelFactory factory_;
    std::unique_ptr<Filter::MotionModel> constant_heading_model_ = factory_.createModel(Filter::ModelType::CONSTANT_HEADING_RATE,&states_);
    Filter::MessageHub hub_(constant_heading_model_.get());
    dynamic_cast<RosCmdVelSource*>(velocity_source_.get())->setMediator(&hub_);
    std::unique_ptr<Filter::Fusion> ekf_ = std::make_unique<Filter::Ekf<geometry_msgs::msg::Twist , sensor_msgs::msg::Imu>>(std::move(constant_heading_model_) , velocity_source_ , imu_source_);
    ekf_->setStates(&states_);
    ekf_->initialize();
    rclcpp::Rate loop_rate_(100);

    while(rclcpp::ok())
    {
        ekf_->predict(rclcpp::Clock().now()); 
        ekf_->update();
        Eigen::MatrixXd states_ = ekf_->getStates();
        executor->spin_some();
        loop_rate_.sleep(); //why is it possible without using spin i can get the cmd_vel topic echoed fine. did they lie to me?!! :)
    }
    rclcpp::shutdown();
}


TEST(Ekf, rviz)
{
    rclcpp::init(0,nullptr);
    std::shared_ptr<VelocitySource<geometry_msgs::msg::Twist>> velocity_source_ = std::make_shared<RosCmdVelSource>();
    std::shared_ptr<ImuSource<sensor_msgs::msg::Imu>> imu_source_ = std::make_shared<RosImuSource>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node( dynamic_cast<RosCmdVelSource*>(velocity_source_.get())->getRosNode());
    executor->add_node(dynamic_cast<RosImuSource*>(imu_source_.get())->getRosNode());

    std::vector<std::string> state_names_{"x" , "y" , "yaw" , "x_dot" , "yaw_dot" , "x_ddot"};
    Filter::StateSpace states_(state_names_);
    Filter::MotionModelFactory factory_;
    std::unique_ptr<Filter::MotionModel> constant_heading_model_ = factory_.createModel(Filter::ModelType::CONSTANT_HEADING_RATE ,&states_);
    Filter::MessageHub hub_(constant_heading_model_.get());
    dynamic_cast<RosCmdVelSource*>(velocity_source_.get())->setMediator(&hub_);
    std::unique_ptr<Filter::Fusion> ekf_ = std::make_unique<Filter::Ekf<geometry_msgs::msg::Twist , sensor_msgs::msg::Imu>>(std::move(constant_heading_model_) , velocity_source_ , imu_source_);
    ekf_->setStates(&states_);
    ekf_->initialize();
    rclcpp::Rate loop_rate_(1000);

    auto visualization_ = std::make_shared<Visualization::Visualization>();
    geometry_msgs::msg::Pose2D pose_;

    while(rclcpp::ok())
    {
        ekf_->predict(rclcpp::Clock().now()); 
        ekf_->update();
        Eigen::MatrixXd states_ = ekf_->getStates();

        pose_.x = states_(0,0);
        pose_.y = states_(1,0);
        pose_.theta = states_(2,0);

        visualization_->addArrow(pose_);
        visualization_->publishArrow();
        visualization_->initialize();

        executor->spin_some();
        loop_rate_.sleep(); //why is it possible without using spin i can get the cmd_vel topic echoed fine. did they lie to me?!! :)
    }
    rclcpp::shutdown();
}