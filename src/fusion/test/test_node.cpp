// MIT License
//
// Copyright (c) 2023 Soheil Espahbodi Nia
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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