#include<gtest/gtest.h>
#include<fusion/ekf.hpp>
#include<fusion/sensor_data.hpp>
#include<fusion/motion_model.hpp>
#include<fusion/motion_model_factory.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include<fusion/visualization.hpp>
#include<fusion/mediator.hpp>

#include<fusion/filter_node.hpp>
TEST(Node , params)
{
    ASSERT_EQ(1,1);
    std::string param_file = "/home/admin/EKF_WS/src/fusion/params/ekf.yaml";

    rclcpp::init(0,nullptr);
    auto node = std::make_shared<Filter::FilterNode>();

    rclcpp::Rate loop_rate(10);
    // node->loadParams();
    // std::vector<bool> bol = node->getStates();
    // for(auto i = bol.begin() ; i!=bol.end() ; i++)
    // {
    //    std::cout<<*i<<"\n";
    // }
    while(rclcpp::ok()) 
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}