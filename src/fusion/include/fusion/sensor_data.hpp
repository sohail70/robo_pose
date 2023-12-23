#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP
#include<rclcpp/rclcpp.hpp>
#include<rclcpp/qos.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include<fusion/mediator.hpp>
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
        Filter::Mediator* mediator_;
        void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr );


    public:
        RosCmdVelSource();
        virtual const geometry_msgs::msg::Twist& getVelocity() const;
        rclcpp::Node::SharedPtr getRosNode();
        void setMediator(Filter::Mediator* );

};

// class RosJointStateVelocitySource: public VelocitySource<double>
// {
//     private:
//         rclcpp::Subscription<geometry_msgs::msg::JointState>::SharedPtr joint_state_sub_;
//         sensor_msgs::msg::JointState joint_state_;
//         rclcpp::Node::SharedPtr node_;
//         void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr );
//     public:
//         RosJointStateVelocitySource();
//         virtual const double& getVelocity() const;
//         rclcpp::Node::SharedPtr getRosNode();


// };


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
        rclcpp::Node::SharedPtr getRosNode();
};

#endif