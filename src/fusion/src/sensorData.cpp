#include<fusion/sensorData.hpp>


RosCmdVelSource::RosCmdVelSource()
{
    node_ = rclcpp::Node::make_shared("cmd_vel_source_node");
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel" , 1 , std::bind(&RosCmdVelSource::CmdCallback , this , std::placeholders::_1));
}

void RosCmdVelSource::CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_ = *msg;
}

const geometry_msgs::msg::Twist& RosCmdVelSource::getVelocity() const
{
    return cmd_vel_;
}


RosImuSource::RosImuSource()
{
    node_ = rclcpp::Node::make_shared("imu_source_node");
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("imu" , 1 , std::bind(&RosImuSource::imuCallback , this , std::placeholders::_1));
}


void RosImuSource::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_ = *msg; 
}