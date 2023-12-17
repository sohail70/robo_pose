#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<sensor_msgs/msg/imu.hpp>
template<typename velType>
class VelocitySource {
    protected:

    public:
        virtual const velType& getVelocity() const = 0 ;
};


class RosCmdVelSource : public VelocitySource<geometry_msgs::msg::Twist>
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        geometry_msgs::msg::Twist cmd_vel_;
        rclcpp::Node::SharedPtr node_;
        
        void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr );

    public:
        RosCmdVelSource();
        virtual const geometry_msgs::msg::Twist& getVelocity() const;
};


template<typename imuType>
class ImuSource{
    protected:

    public:
        virtual const imuType& getImuData() const = 0;

};


class RosImuSource: public ImuSource<sensor_msgs::msg::Imu>{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        sensor_msgs::msg::Imu imu_;
        rclcpp::Node::SharedPtr node_;
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr );
    public:
       RosImuSource();
       virtual const sensor_msgs::msg::Imu& getImuData() const override;
};

#endif