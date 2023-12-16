#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include<iostream>
// #include<Eigen/Dense>
#include<cmath>
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
namespace Filter{
    struct Position{
        double x, y, z;
    };
    struct Angle{
        double roll, pitch, yaw;
    };
    struct Velocity{
        double x_dot,y_dot,z_dot;
    };
    struct AngularVelocity{
        double roll_dot, pitch_dot , yaw_dot;
    };
    class MotionModel{
            protected:
                Position position_;
                Angle angle_;
                Velocity velocity_;
                AngularVelocity angular_velocity_;
                rclcpp::Time previous_time_;
            public:
                MotionModel();
                virtual ~MotionModel();
                virtual void init() = 0;
                virtual void update(const rclcpp::Time& ) = 0;
                virtual void setVelocity(const Filter::Velocity& ) = 0;
                virtual void setAngularVelocity(const Filter::AngularVelocity& ) = 0;
                virtual void setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& ) = 0;
                virtual rclcpp::Time getPrevTime() = 0;
        };
} //namespace Filter
#endif