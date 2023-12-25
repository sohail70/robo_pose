#include<fusion/state_space.hpp>

namespace Filter{
    StateSpace::StateSpace(std::vector<std::string> state_names_)
    {
        std::cout<<"Ctor of state space \n";
        for(auto& state_name_ :state_names_)
        {
            states_[state_name_] = 0.0;
            state_order_.push_back(state_name_);
        }

    }

    void StateSpace::updateStates(std::vector<double> values_)
    {
        int index = 0;
        for(auto&  state_ : state_order_)
        {
            states_[state_] = values_[index];
            index++;
        }
    }

    std::unordered_map<std::string,double>& StateSpace::getStates()
    {
        return states_;
    }

    std::vector<std::string>& StateSpace::getStateOrder() {
        return state_order_;
    }

};