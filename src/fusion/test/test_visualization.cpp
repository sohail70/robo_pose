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
    // visual_.addPointToLineStrip(point_1_);
    // visual_.addPointToLineStrip(point_2_);
    // visual_.addLineSegment(point_3_ , point_4_);
    while (rclcpp::ok())
    {
        visual_.publishPoints();
        visual_.publishLineStrip();
        visual_.publishLineList();
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

#include<thread>

Filter::Position position_; //instead of global variables you can also create a class with these variables with update and get function and give the reference to the obj of that class to visualzation thread as the second argument
Filter::Angle angle_;

void visualization_thread_()
{
    rclcpp::Rate loop_rate_(100);
    while (rclcpp::ok())
    {
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("Thread") , "In the thread");
        //Animtion
        // plt::clf();
        std::vector<double> x, y, dx, dy;
        x.push_back(position_.x);
        y.push_back(position_.y);
        dx.push_back(cos(angle_.yaw)); // X component of the vector
        dy.push_back(sin(angle_.yaw)); // Y component of the vector 
        plt::plot(x, y, "r.");
        plt::quiver(x, y, dx, dy);
        plt::pause(0.0000001);
        loop_rate_.sleep();
    }
    
}

TEST(visualizationTest , cmdVelTopicTestMultiThread)
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

    std::thread vis_thread_{visualization_thread_};

    while(rclcpp::ok())
    {
        
        cur_time_ = rclcpp::Clock().now(); // *
        constant_heading_rate_->setVelAndAngVelFromTwist(cmd);
        constant_heading_rate_->update(rclcpp::Clock().now());
        position_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getPosition();
        angle_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getAngle();

        rclcpp::spin_some(node_);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("myLogger") , cmd.linear.x);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rate logger") , 1/(cur_time_-pre_time_).seconds()); // *
        pre_time_ = cur_time_; // *
        loop_rate_.sleep();

    }

    if (vis_thread_.joinable())
        vis_thread_.join();

    rclcpp::shutdown();

}


#include <condition_variable>

struct ThreadParams {
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Visualization::Visualization> visualization_;
    std::mutex visualization_mutex_;
    std::condition_variable cv_;
    bool ready;
};

void rviz_marker(ThreadParams* params_) {
    rclcpp::Rate loop_rate(1000);
    while (rclcpp::ok()) {
            {
                std::unique_lock<std::mutex> lock_(params_->visualization_mutex_);
                params_->cv_.wait(lock_ ,[params_]{return params_->ready;});
                params_->visualization_->publishPoints();
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
                params_->visualization_->publishLineStrip();
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
                params_->visualization_->publishLineList();
                // RCLCPP_INFO_STREAM(rclcpp::get_logger("Y") , "PROCESSED");
                params_->ready = false;
            }
            params_->cv_.notify_one(); //notifies the delete operation
 

        loop_rate.sleep();
    }
}


TEST(visualizationTest , cmdVelTopicTestRvizMarkers)
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

    auto visualization_ = std::make_shared<Visualization::Visualization>();
    geometry_msgs::msg::Point point_;

    ThreadParams params_;
    params_.visualization_ = visualization_;
    params_.node_ = node_;

    std::thread rviz_marker_(rviz_marker , &params_);
    while(rclcpp::ok())
    {
        cur_time_ = rclcpp::Clock().now(); // *
        constant_heading_rate_->setVelAndAngVelFromTwist(cmd);
        constant_heading_rate_->update(rclcpp::Clock().now());
        position_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getPosition();
        angle_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getAngle();
  

        point_.x = position_.x;
        point_.y = position_.y;
        point_.z = position_.z;

        {
            std::lock_guard<std::mutex> lock_(params_.visualization_mutex_);
            visualization_->addPoint(point_);
            params_.ready = true;
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("X" ), "ADDED");
        }
        params_.cv_.notify_one(); //notifies the other thread (rviz_marker)
        {
            std::unique_lock<std::mutex> lock_(params_.visualization_mutex_);
            params_.cv_.wait(lock_, [&params_]{ return !params_.ready; });
            visualization_->deletePoint(0);
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("X" ), "DELETED");
        }

  
        rclcpp::spin_some(node_);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("RATE") , 1/(cur_time_.seconds() - pre_time_.seconds()));
        pre_time_ = cur_time_; // *
        loop_rate_.sleep();
    }
    rviz_marker_.join();

    rclcpp::shutdown();

}




TEST(visualizationTest , cmdVelTopicTestRvizMarkersSimple)
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

    auto visualization_ = std::make_shared<Visualization::Visualization>();
    geometry_msgs::msg::Point point_;


    while(rclcpp::ok())
    {
        cur_time_ = rclcpp::Clock().now(); // *
        constant_heading_rate_->setVelAndAngVelFromTwist(cmd);
        constant_heading_rate_->update(rclcpp::Clock().now());
        position_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getPosition();
        angle_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getAngle();
  

        point_.x = position_.x;
        point_.y = position_.y;
        point_.z = position_.z;

        visualization_->addPoint(point_);
        visualization_->publishPoints();
        visualization_->deletePoint(0);

  
        rclcpp::spin_some(node_);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("RATE") , 1/(cur_time_.seconds() - pre_time_.seconds()));
        pre_time_ = cur_time_; // *
        loop_rate_.sleep();
    }

    rclcpp::shutdown();

}

TEST(visualizationTest , cmdVelTopicTestRvizMarkersArrow)
{
    rclcpp::init(0,nullptr);
    auto node_ = rclcpp::Node::make_shared("subcmd");
    geometry_msgs::msg::Twist cmd;
    auto sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel" ,1, [&cmd](const geometry_msgs::msg::Twist::SharedPtr msg){cmd_callback(msg,cmd);});
    // auto sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel" ,1, std::bind(cmd_callback , std::placeholders::_1 , std::ref(cmd))); //this version needs a std::reference_warpper<geometr...Twist> on the args of the callback and also needs a cmd.get() in the body of the callback
    Filter::MotionModelFactory factory_;
    std::unique_ptr<Filter::MotionModel> constant_heading_rate_ =  factory_.createModel(Filter::ModelType::CONSTANT_HEADING_RATE);

    rclcpp::Rate loop_rate_(100);
    auto cur_time_ = rclcpp::Clock().now(); // *
    auto pre_time_ = cur_time_; // *

    auto visualization_ = std::make_shared<Visualization::Visualization>();
    geometry_msgs::msg::Pose2D pose_;


    while(rclcpp::ok())
    {
        cur_time_ = rclcpp::Clock().now(); // *
        constant_heading_rate_->setVelAndAngVelFromTwist(cmd);
        constant_heading_rate_->update(rclcpp::Clock().now());
        position_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getPosition();
        angle_ = dynamic_cast<Filter::ConstantHeadingRate*>(constant_heading_rate_.get())->getAngle();
  

        pose_.x = position_.x;
        pose_.y = position_.y;
        pose_.theta = angle_.yaw;

        visualization_->addArrow(pose_);
        visualization_->publishArrow();
        visualization_->initialize();

  
        rclcpp::spin_some(node_);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("RATE") , 1/(cur_time_.seconds() - pre_time_.seconds()));
        pre_time_ = cur_time_; // *
        loop_rate_.sleep();
    }

    rclcpp::shutdown();

}
