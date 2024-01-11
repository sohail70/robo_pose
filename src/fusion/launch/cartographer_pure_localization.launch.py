# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim



#  ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "map_url: '/home/admin/ws/src/t32/maps/map.yaml'" 



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import TimerAction
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    turtlebot3_cartographer_prefix = get_package_share_directory('fusion')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_pure_localization.lua')
    map_path = LaunchConfiguration('map_path',
                                    default=os.path.join(turtlebot3_cartographer_prefix ,"config/map.pbstream" ) )

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('fusion'),
                                   'config', 'rviz2.rviz')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Map server
    map_server_config_path = os.path.join(
        get_package_share_directory('fusion'),
        'maps',
        'map.yaml'
    )
    lifecycle_nodes = ['map_server']
    #use_sim_time = True
    autostart = True
    print("SSSSSSSSSSSSSSSSSSSSSSS",map_server_config_path),

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'map_path',
            default_value= map_path,
            description='map path'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-collect-metrics',
                       '-load_state_filename', map_path,
                       ]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),


        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
        #                       'publish_period_sec': publish_period_sec}.items(),
        # ),
            Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            #arguments=['-d', rviz_config_dir],
            arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),


                              
                     
            Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

            Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_server_config_path}]), 
         TimerAction(period=1.0,
            actions=[    
            ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/map_server/load_map ",
                "nav2_msgs/srv/LoadMap ",
                '"{map_url: \'/home/admin/EKF_WS/src/fusion/maps/map.yaml\'}"' #this works but below doesn't!!!
                # '"{map_url: map_server_config_path}"'
            ]],
            shell=True)
            ])
            

    
    ])
