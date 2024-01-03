#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include<iostream>
#include<Eigen/Dense>
#include<cmath>
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<fusion/state_space.hpp>
#include<autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

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
                rclcpp::Duration dt_;
                rclcpp::Time previous_time_;
                std::shared_ptr<StateSpace> states_;
            public:
                MotionModel();
                virtual ~MotionModel();
                virtual void init() = 0;
                virtual Eigen::MatrixXd update(const rclcpp::Time& ) = 0;
                virtual autodiff::VectorXreal propagate(const autodiff::VectorXreal&)=0;
                virtual void setVelocity(const Filter::Velocity& ) = 0;
                virtual void setAngularVelocity(const Filter::AngularVelocity& ) = 0;
                virtual void setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& ) = 0;
                virtual void setStates(std::shared_ptr<StateSpace> states_) = 0;
                virtual Eigen::MatrixXd getJacobian() = 0;
                virtual rclcpp::Duration getDt() = 0;
        };
} //namespace Filter
#endif