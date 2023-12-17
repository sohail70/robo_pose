#include<gtest/gtest.h>
#include<fusion/ekf.hpp>
#include<fusion/sensor_data.hpp>
#include<fusion/motion_model.hpp>
#include<fusion/motion_model_factory.hpp>
TEST(Ekf , prediction)
{
    ASSERT_EQ(1,1);
    rclcpp::init(0,nullptr);
    std::shared_ptr<VelocitySource<geometry_msgs::msg::Twist>> velocity_source_ = std::make_shared<RosCmdVelSource>();
    std::shared_ptr<ImuSource<sensor_msgs::msg::Imu>> imu_source_ = std::make_shared<RosImuSource>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // executor->add_node( dynamic_cast<RosCmdVelSource*>(velocity_source_.get())->getRosNode());
    executor->add_node(dynamic_cast<RosImuSource*>(imu_source_.get())->getRosNode());

    Filter::MotionModelFactory factory_;
    std::unique_ptr<Filter::MotionModel> constant_heading_model_ = factory_.createModel(Filter::ModelType::CONSTANT_HEADING_RATE);
    std::unique_ptr<Filter::Fusion> ekf_ = std::make_unique<Filter::Ekf<geometry_msgs::msg::Twist , sensor_msgs::msg::Imu>>(std::move(constant_heading_model_) , velocity_source_ , imu_source_);
    ekf_->initialize();
    rclcpp::Rate loop_rate_(1);

    while(rclcpp::ok())
    {
        ekf_->predict(rclcpp::Clock().now()); 
        ekf_->update();
        executor->spin_some();
        loop_rate_.sleep(); //why is it possible without using spin i can get the cmd_vel topic echoed fine. did they lie to me?!! :)
    }
    rclcpp::shutdown();
}