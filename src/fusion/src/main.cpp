#include<rclcpp/rclcpp.hpp>
#include<fusion/filter_node.hpp>
int main(int argc , char* argv[])
{
    rclcpp::init(argc , argv);

    auto node = std::make_shared<Filter::FilterNode>();

    rclcpp::Rate loop_rate(10);
    node->loadParams();
    auto bol = node->getStates();
    for(auto i = bol.begin() ; i!=bol.end() ; i++)
    {
       RCLCPP_INFO(node->get_logger(),"alo %d" , static_cast<int>(*i));
    }
    // while(rclcpp::ok()) 
    // {
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep();
    // }
    rclcpp::shutdown();
}