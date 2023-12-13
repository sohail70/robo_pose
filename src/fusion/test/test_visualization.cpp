#include<fusion/visualization.hpp>
#include<gtest/gtest.h>

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