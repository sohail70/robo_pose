#ifndef STATE_SPACE_HPP
#define STATE_SPACE_HPP

#include<unordered_map>
#include<vector>
#include<iostream>
namespace Filter{
    class  StateSpace
    {
        public:
            StateSpace(std::vector<std::string> );
            void updateStates(std::vector<double> );
            std::unordered_map<std::string,double>& getStates();
            std::vector<std::string>& getStateOrder();

        public:
            std::unordered_map<std::string,double> states_;
            std::vector<std::string> state_order_; 


    };



} //namespace Filter






#endif