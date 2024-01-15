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

#include<fusion/sensor_data.hpp>


RosCmdVelSource::RosCmdVelSource()
{
    node_ = rclcpp::Node::make_shared("cmd_vel_source_node");
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel" , 1 , std::bind(&RosCmdVelSource::cmdCallback , this , std::placeholders::_1));
}

void RosCmdVelSource::setMediator(Filter::Mediator* mediator_)
{
    this->mediator_ = mediator_;
}

void RosCmdVelSource::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_ = *msg;
    Filter::Message message;
    message.type = Filter::Message::MessageType::VELOCITY_UPDATE;
    message.velocityData = *msg;
    if(mediator_)
    {
        mediator_->notifyMessage(message);
    }
}

const geometry_msgs::msg::Twist& RosCmdVelSource::getVelocity() const
{
    return cmd_vel_;
}



rclcpp::Node::SharedPtr RosCmdVelSource::getRosNode()
{
    return node_;
}


RosImuSource::RosImuSource()
{
    node_ = rclcpp::Node::make_shared("imu_source_node");
    rclcpp::QoS qos_ = rclcpp::SensorDataQoS();
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("imu" , qos_ , std::bind(&RosImuSource::imuCallback , this , std::placeholders::_1));
}


void RosImuSource::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_ = *msg; 
}

const sensor_msgs::msg::Imu& RosImuSource::getImuData() const
{
    return imu_;
}

rclcpp::Node::SharedPtr RosImuSource::getRosNode()
{
    return node_;
}