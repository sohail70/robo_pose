#include<fusion/visualization.hpp>



namespace Visualization{
    Visualization::Visualization(): Node("VisualizationNode")
    {
        std::cout<<"Hey \n";
        std_msgs::msg::ColorRGBA color;
        color.a = 1; color.g = 1;

        points_.color = color;    
        points_.header.frame_id = "map";
        points_.header.stamp = this->now();
        points_.pose.orientation.w = 1.0;
        // points_.id = id;
        points_.action = visualization_msgs::msg::Marker::ADD;
        points_.scale.x = 0.1;
        points_.scale.y = 0.1;
        points_.type = visualization_msgs::msg::Marker::POINTS;

        color.a = 1; color.r = 1;
        line_strip_.color = color;    
        line_strip_.header.frame_id = "map";
        line_strip_.header.stamp = this->now(); 
        line_strip_.pose.orientation.w = 1.0;
        // line_strip_.id = id;
        line_strip_.action = visualization_msgs::msg::Marker::ADD;
        line_strip_.scale.x = 0.1;
        line_strip_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        
        color.a = 1; color.b = 1;
        line_list_.color = color;    
        line_list_.header.frame_id = "map";
        line_list_.header.stamp = this->now();
        line_list_.pose.orientation.w = 1.0;
        // line_list_.id = id;
        line_list_.action = visualization_msgs::msg::Marker::ADD;
        line_list_.scale.x = 0.1;
        line_list_.type = visualization_msgs::msg::Marker::LINE_LIST;
        // id++;

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

    void Visualization::initialize()
    {
        points_.points.clear();
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


    void Visualization::publish()
    {
        marker_pub_->publish(points_);
        marker_pub_->publish(line_strip_);
        marker_pub_->publish(line_list_);
    }


} //namespace Visualization