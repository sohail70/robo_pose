#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include<rclcpp/rclcpp.hpp>
#include<visualization_msgs/msg/marker.hpp>
#include<geometry_msgs/msg/point.hpp>
#include<std_msgs/msg/color_rgba.hpp>

namespace Visualization{
    class Visualization : public rclcpp::Node{
        public:
            Visualization();
            void addPoint(geometry_msgs::msg::Point );
            void deletePoint(int );
            void addPointToLineStrip(geometry_msgs::msg::Point );
            void removePointToLineStrip(int );
            void addLineSegment(geometry_msgs::msg::Point , geometry_msgs::msg::Point);
            void removeLineSegment(int );
            void initialize();
            void publish();
        private:
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
            visualization_msgs::msg::Marker points_;
            visualization_msgs::msg::Marker line_strip_;
            visualization_msgs::msg::Marker line_list_;
            // static int id;
    };
    // int Visualization::id = 0;


    // class LineStrip: public rclcpp::Node{

    // };

    // class LineList{


    // };

} //namespace Visualization


#endif
