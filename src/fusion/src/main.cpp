#include<rclcpp/rclcpp.hpp>
#include<fusion/filter_node.hpp>
int main(int argc , char* argv[])
{
    rclcpp::init(argc , argv);

    rclcpp::NodeOptions options_;
    options_.allow_undeclared_parameters(false);
    auto node = std::make_shared<Filter::FilterNode>(options_);

    rclcpp::Rate loop_rate(10);
    // auto names = node->getStateSpace();
    // for(auto i = names.begin() ; i!=names.end() ; i++)
    // {
    //    RCLCPP_INFO(node->get_logger(),"alo %s" , (*i).c_str());
    // }
    // while(rclcpp::ok()) 
    // {
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep();
    // }
    rclcpp::shutdown();
}