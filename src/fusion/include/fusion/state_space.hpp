#ifndef STATE_SPACE_HPP
#define STATE_SPACE_HPP

#include<unordered_map>
#include<vector>
#include<iostream>
#include<autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
namespace Filter{
    class  StateSpace
    {
        public:
            StateSpace(std::vector<std::string> );
            void updateStates(std::vector<double> );
            autodiff::VectorXreal& getStates();
            std::unordered_map<std::string,int>& getStateOrder();

        public:
            std::unordered_map<std::string,int> states_name_;
            // std::vector<std::string> state_order_; 
            autodiff::VectorXreal states_;


    };



} //namespace Filter






#endif