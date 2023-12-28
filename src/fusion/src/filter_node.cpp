#include<fusion/filter_node.hpp>

namespace Filter{
    FilterNode::FilterNode():Node("Filter")
    {
        this->declare_parameter<std::vector<bool>>("states");
        // this->declare_parameter<double>("my_parameter_double", 0.0);
        // this->declare_parameter<std::string>("my_parameter_string", "");

    }

    void FilterNode::loadParams(){
        this->get_parameter("states" , states_);
    }

    std::vector<bool> FilterNode::getStates()
    {
        return states_;
    }
} // namespace Filter