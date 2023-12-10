#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include<iostream>
// #include<Eigen/Dense>
#include<cmath>
#include<rclcpp/rclcpp.hpp>
namespace Filter{
    class MotionModel{
            public:
                MotionModel();
                virtual ~MotionModel();
                virtual void init() = 0;
                virtual void update(const rclcpp::Time& ) = 0;

            protected:
                struct Position{
                    double x, y, z;
                };
                Position position_;

                struct Angle{
                    double roll, pitch, yaw;
                }; 
                Angle angle_;

                struct Velocity{
                    double x_dot,y_dot,z_dot;
                };
                Velocity velocity_;

                struct AngularVelocity{
                    double roll_dot, pitch_dot , yaw_dot;
                };
                AngularVelocity angular_velocity_;

                rclcpp::Time previous_time_;
        };
} //namespace Filter
#endif