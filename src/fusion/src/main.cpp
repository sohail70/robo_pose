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

#include<rclcpp/rclcpp.hpp>
#include<fusion/filter_node.hpp>
int main(int argc , char* argv[])
{
    rclcpp::init(argc , argv);

    rclcpp::NodeOptions options_;
    options_.allow_undeclared_parameters(false);
    auto filter_node_ = std::make_shared<Filter::FilterNode>(options_);

    // rclcpp::Rate loop_rate(10);
    // auto names = node->getStateSpace();
    // for(auto i = names.begin() ; i!=names.end() ; i++)
    // {
    //    RCLCPP_INFO(node->get_logger(),"alo %s" , (*i).c_str());
    // }
    // while(rclcpp::ok()) 
    // {
        // RCLCPP_INFO(filter_node_->get_logger(),"working");
        // rclcpp::spin_some(filter_node_);
        // loop_rate.sleep();
    // }
    rclcpp::spin(filter_node_);
    rclcpp::shutdown();
}