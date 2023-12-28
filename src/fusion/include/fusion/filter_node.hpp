#ifndef FILTER_NODE_HPP
#define FILTER_NODE_HPP
#include<rclcpp/rclcpp.hpp>
#include<autodiff/forward/real.hpp>
#include<autodiff/forward/real/eigen.hpp>
namespace Filter{
    class FilterNode: public rclcpp::Node{
        public:
            FilterNode();
            void loadParams();
            std::vector<bool> getStates();

        private:
            std::vector<bool> states_;
            std::vector<double>  process_noise_covariance_;
            autodiff::MatrixXreal Q; 
            autodiff::VectorXreal S;

    };
} // namespace Filter


#endif