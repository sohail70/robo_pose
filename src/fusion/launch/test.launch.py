import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command , LaunchConfiguration

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="fusion").find("fusion")
    default_model_path = os.path.join(pkg_share , "urdf/robot.urdf.xacro ")
    world_arg = DeclareLaunchArgument('world', default_value='worlds/willowgarage.world', description='Gazebo world file')
    package_name = 'fusion'  # Replace with your actual package name
    config_file_name = 'params.yaml'
    pkg_dir = get_package_share_directory(package_name)
    config = os.path.join(pkg_dir, 'params', 'gazebo_config.yaml')
    gazebo_arg = DeclareLaunchArgument('gazebo', default_value=config, description='Path to Gazebo configuration file')

    environment_variables = {
        'GAZEBO_MODEL_PATH': os.path.join(get_package_share_directory('fusion'), 'worlds/willowgarage.world')
    }
    # Define Gazebo launch process
    gazebo_cmd = [
        'gazebo',
        '--verbose',
        LaunchConfiguration('world'),
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        '--pause',
        '--ros-args',
        '--params-file', LaunchConfiguration('gazebo')
    ]
    gazebo = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen',
        additional_env=environment_variables
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': Command(['xacro ',LaunchConfiguration('model')])} , {'use_sim_time':use_sim_time}]
    )

    joint_state_publisher = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{'use_sim_time':use_sim_time}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))

    )

    joint_state_publisher_gui = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{'use_sim_time':use_sim_time}],
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui"))
    )

    spawn_robot = launch_ros.actions.Node(package="gazebo_ros" , executable="spawn_entity.py" , output = "screen" , 
                                            arguments=[
                                                "-topic" , "robot_description",
                                                "-entity", "t32",
                                                "-x" , "0.0" , "-y" , "0.0" , "-z" , "0.",
                                                "-R" , "0.0" , "-P" , "0.0" , "-Y" , "0.0",
                                            ]
                                          )
    # Create the launch description
    ld = LaunchDescription()

    ld.add_action(launch.actions.DeclareLaunchArgument(name='gui' , default_value='False' , description="Flag to enable joint_state_publisher_gui"))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path , description='Absolute path to robot urdf file'))

    ld.add_action(world_arg)
    ld.add_action(gazebo_arg)
    ld.add_action(gazebo)
    ld.add_action(joint_state_publisher) #in va paeeni age bashan robot dar gazebo nemiad!!!!
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    

    return ld