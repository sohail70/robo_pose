#include<fusion/visualization.hpp>



namespace Visualization{
    Visualization::Visualization(): Node("VisualizationNode")
    {
        std_msgs::msg::ColorRGBA color;
        color.a = 1; color.g = 1;

        points_.color = color;    
        points_.header.frame_id = "odom";
        points_.ns = "points";
        points_.pose.orientation.w = 1.0;
        points_.id = 1;
        points_.action = visualization_msgs::msg::Marker::ADD;
        points_.scale.x = 0.1;
        points_.scale.y = 0.1;
        points_.type = visualization_msgs::msg::Marker::POINTS;

        color.a = 1; color.r = 1;
        line_strip_.color = color;    
        line_strip_.header.frame_id = "odom";
        line_strip_.ns = "linestrip";
        line_strip_.pose.orientation.w = 1.0;
        line_strip_.id = 2;
        line_strip_.action = visualization_msgs::msg::Marker::ADD;
        line_strip_.scale.x = 0.1;
        line_strip_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        
        color.a = 1; color.b = 1;
        line_list_.color = color;    
        line_list_.header.frame_id = "odom";
        line_list_.ns = "linelist";
        line_list_.pose.orientation.w = 1.0;
        line_list_.id = 3;
        line_list_.action = visualization_msgs::msg::Marker::ADD;
        line_list_.scale.x = 0.1;
        line_list_.type = visualization_msgs::msg::Marker::LINE_LIST;
        // id++;

        color.a =1; color.b =1;
        arrow_.color = color;
        arrow_.header.frame_id = "odom"; // Change the frame_id to match your frame
        arrow_.ns = "arrows";
        arrow_.id = 4;
        arrow_.type = visualization_msgs::msg::Marker::ARROW;
        arrow_.action = visualization_msgs::msg::Marker::ADD;
        arrow_.scale.x = 1.0;
        arrow_.scale.y = 0.1;
        arrow_.scale.z = 0.1;

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker",2);
    }

    void Visualization::addPoint(geometry_msgs::msg::Point point_)
    {
        points_.points.push_back(point_);
    }

    void Visualization::deletePoint(int index_)
    {
        points_.points.erase(points_.points.begin()+index_);
    }

    void Visualization::clear()
    {
        points_.points.clear();
        line_strip_.points.clear();
        line_list_.points.clear();
        arrow_.points.clear();
    }

    void Visualization::addPointToLineStrip(geometry_msgs::msg::Point point_)
    {
        line_strip_.points.push_back(point_);
    }

    void Visualization::removePointToLineStrip(int index_)
    {
        line_strip_.points.erase(line_strip_.points.begin()+index_);
    }

    void Visualization::addLineSegment(geometry_msgs::msg::Point point_1_ , geometry_msgs::msg::Point point_2_)
    {
        line_list_.points.push_back(point_1_);
        line_list_.points.push_back(point_2_);
    }

    void Visualization::removeLineSegment(int index_)
    {
        line_list_.points.erase(line_list_.points.begin() , line_list_.points.begin()+index_+1);
    }

    void Visualization::addArrow(geometry_msgs::msg::Pose pose_)
    {
        arrow_.pose = pose_;
    }

    void Visualization::addArrow(geometry_msgs::msg::Pose2D pose_)
    {
        tf2::Quaternion quat;
        quat.setRPY(0.0,0.0,pose_.theta);
        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = quat.getX(); 
        quat_msg.y = quat.getY(); 
        quat_msg.z = quat.getZ(); 
        quat_msg.w = quat.getW(); 
        arrow_.pose.position.x = pose_.x;
        arrow_.pose.position.y = pose_.y;
        arrow_.pose.orientation = quat_msg;
    }
    void Visualization::publishPoints() {
        points_.header.stamp = this->now();
        marker_pub_->publish(points_);
    }

    void Visualization::publishLineStrip() {
        line_strip_.header.stamp = this->now();
        marker_pub_->publish(line_strip_);
    }

    void Visualization::publishLineList() {
        line_list_.header.stamp = this->now();
        marker_pub_->publish(line_list_);
    }
    
    void Visualization::publishArrow(const rclcpp::Time& t_) {
        // arrow_.header.stamp = this->now();
        arrow_.header.stamp = t_;
        marker_pub_->publish(arrow_);
    }

} //namespace Visualization