#include<fusion/state_space.hpp>

namespace Filter{
    StateSpace::StateSpace(std::vector<std::string> state_names_)
    {
        states_.resize(state_names_.size());
        std::cout<<"Ctor of state space \n";
        int index = 0;
        for(auto& state_name_ :state_names_)
        {
            states_[index] = 0.0;
            states_name_[state_name_] = index;
            index++;
        }

    }

    void StateSpace::updateStates(std::vector<double> values_)
    {
        int index = 0;
        for(auto&  value_ : values_)
        {
            states_[index] = values_[index];
            index++;
        }
    }

    autodiff::VectorXreal& StateSpace::getStates()
    {
        return states_;
    }

    std::unordered_map<std::string,int>& StateSpace::getStateOrder() {
        return states_name_;
    }

};