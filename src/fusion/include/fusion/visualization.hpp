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

#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include<rclcpp/rclcpp.hpp>
#include<visualization_msgs/msg/marker.hpp>
#include<geometry_msgs/msg/point.hpp>
#include<std_msgs/msg/color_rgba.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include<mutex>
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
            void addArrow(geometry_msgs::msg::Pose );
            void addArrow(geometry_msgs::msg::Pose2D );
            void clear();
            void publishPoints();
            void publishLineStrip();
            void publishLineList();
            void publishArrow(const rclcpp::Time&);
            
        public:
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
            visualization_msgs::msg::Marker points_;
            visualization_msgs::msg::Marker line_strip_;
            visualization_msgs::msg::Marker line_list_;
            visualization_msgs::msg::Marker arrow_;

            // static int id;
    };
    // int Visualization::id = 0;


    // class LineStrip: public rclcpp::Node{

    // };

    // class LineList{


    // };

} //namespace Visualization


#endif
