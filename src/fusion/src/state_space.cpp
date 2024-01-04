#include<fusion/state_space.hpp>
#include<rclcpp/rclcpp.hpp>
namespace Filter{
    StateSpace::StateSpace(std::vector<std::string> state_names_)
    {
        states_.resize(state_names_.size());
        state_names_.resize(state_names_.size());
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
    // TODO: This should be const ref but for now im using it to update the states_ in the constant heading rate function and ekf so it'd be best to use updatesStates to update things!
    autodiff::VectorXreal& StateSpace::getStates()
    {
        return states_;
    }

    const std::unordered_map<std::string,int>& StateSpace::getStateOrder() {
        return states_name_;
    }

};