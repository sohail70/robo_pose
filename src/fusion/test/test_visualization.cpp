#include<fusion/visualization.hpp>
#include<gtest/gtest.h>
#include<fusion/motion_model_factory.hpp>
#include<fusion/motion_model.hpp>
#include<matplotlib-cpp/matplotlibcpp.h>

TEST(visualizationTest , rvizMarkers)
{
    ASSERT_EQ(1,1);
    rclcpp::init(0 , nullptr);
    Visualization::Visualization visual_;
    geometry_msgs::msg::Point point_1_, point_2_,point_3_,point_4_;
    point_1_.x = 0; point_1_.y = 0; point_1_.z = 0;
    point_2_.x = 1; point_2_.y = 1; point_2_.z = 0;
    point_3_.x = 2; point_3_.y = 2; point_3_.z = 0;
    point_4_.x = 4; point_4_.y = 4; point_4_.z = 0;

    visual_.addPoint(point_1_);
    visual_.addPoint(point_2_);
    visual_.addPointToLineStrip(point_1_);
    visual_.addPointToLineStrip(point_2_);
    visual_.addLineSegment(point_3_ , point_4_);
    while (rclcpp::ok())
    {
        visual_.publish();
    }
     
}



void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg , geometry_msgs::msg::Twist& cmd)
{
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("myLogger") , "soheil");
    cmd = *msg;
}

namespace plt = matplotlibcpp;

TEST(visualizationTest , cmdVelTopicTest)
{
    rclcpp::init(0,nullptr);
    auto node_ = rclcpp::Node::make_shared("subcmd");
    geometry_msgs::msg::Twist cmd;
    auto sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel" ,1, [&cmd](const geometry_msgs::msg::Twist::SharedPtr msg){cmd_callback(msg,cmd);});
    // auto sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel" ,1, std::bind(cmd_callback , std::placeholders::_1 , std::ref(cmd))); //this version needs a std::reference_warpper<geometr...Twist> on the args of the callback and also needs a cmd.get() in the body of the callback
    Filter::MotionModelFactory factory_;
    std::unique_ptr<Filter::MotionModel> constant_heading_rate_ =  factory_.createModel(Filter::ModelType::CONSTANT_HEADING_RATE);

    rclcpp::Rate loop_rate_(1000);
    auto cur_time_ = rclcpp::Clock().now(); // *
    auto pre_time_ = cur_time_; // *
    while(rclcpp::ok())
    {
        // plt::clf();
        cur_time_ = rclcpp::Clock().now(); // *
        constant_heading_rate_->setVelAndAngVelFromTwist(cmd);
        constant_heading_rate_->update(rclcpp::Clock().now());
        Filter::Position position_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getPosition();
        Filter::Angle angle_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getAngle();


        rclcpp::spin_some(node_);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("myLogger") , cmd.linear.x);

        //Animtion
        std::vector<double> x, y, dx, dy;
        x.push_back(position_.x);
        y.push_back(position_.y);
        dx.push_back(cos(angle_.yaw)); // X component of the vector
        dy.push_back(sin(angle_.yaw)); // Y component of the vector 
        plt::plot(x, y, "r.");
        plt::quiver(x, y, dx, dy);
        plt::pause(0.0000001);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rate logger") , 1/(cur_time_-pre_time_).seconds()); // *
        pre_time_ = cur_time_; // *

        loop_rate_.sleep();

    }
    rclcpp::shutdown();

}