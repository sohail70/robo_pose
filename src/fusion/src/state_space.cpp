#include<fusion/state_space.hpp>

namespace Filter{
    StateSpace::StateSpace(std::vector<std::string> state_names_)
    {
        std::cout<<"Ctor of state space \n";
        for(auto& state_name_ :state_names_)
        {
            states_[state_name_] = 0.0;
        }

    }

    void StateSpace::updateStates()
    {
        for(auto& state_ : states_)
        {
            std::cout<<state_.first<<"^^^^" <<state_.second<<"\n";
        }
    }

};