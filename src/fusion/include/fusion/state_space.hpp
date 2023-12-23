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
            void updateStates();
            std::unordered_map<std::string,double>& getStates();

        public:
            std::unordered_map<std::string,double> states_;

    };



} //namespace Filter






#endif