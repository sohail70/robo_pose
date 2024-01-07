#ifndef OBSERVATIONS_HPP
#define OBSERVATIONS_HPP            
#include<rclcpp/rclcpp.hpp>
#include<autodiff/forward/real.hpp>
namespace Filter{
    struct Observations{
        rclcpp::Time time_;
        autodiff::VectorXreal states_;
        autodiff::MatrixXreal H;
        bool operator<(const Observations& other) const{
            return time_>other.time_; //lower time has priority
        }
    };
} //namespace Filter
#endif