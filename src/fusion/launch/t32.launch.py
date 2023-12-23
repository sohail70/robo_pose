import launch
import launch_ros
import os
from launch.substitutions import Command , LaunchConfiguration

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="fusion").find("fusion")
    default_model_path = os.path.join(pkg_share , "urdf/robot.urdf.xacro ")
    # default_model_path = os.path.join(pkg_share , "urdf/t32.urdf.xacro")
    gazebo_share = launch_ros.substitutions.FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    args = launch.actions.DeclareLaunchArgument("world" , default_value=os.path.join(pkg_share , "worlds" , "empty.world.xml"))
    gazebo = launch.actions.IncludeLaunchDescription(os.path.join(gazebo_share , "launch" , "gazebo.launch.py"))
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


    diff_drive_spawner = launch_ros.actions.Node(package="controller_manager" , executable="spawner" , arguments=['diff_cont'] , remappings=[('diff_cont/cmd_vel_unstamped', 'cmd_vel')])
    joint_broad_spawner = launch_ros.actions.Node(package="controller_manager" , executable="spawner" , arguments=['joint_broad'], remappings=[('diff_cont/cmd_vel_unstamped', 'cmd_vel')])






    ld = launch.LaunchDescription()
    ld.add_action(launch.actions.DeclareLaunchArgument(name='gui' , default_value='True' , description="Flag to enable joint_state_publisher_gui"))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path , description='Absolute path to robot urdf file'))
    ld.add_action(args)
    ld.add_action(gazebo)
    # ld.add_action(joint_state_publisher) #in va paeeni age bashan robot dar gazebo nemiad!!!!
    # ld.add_action(joint_state_publisher_gui)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    
    #if you are using control.urdf.xacro in your urdf file use the following, otherwise comment them
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)

    # ld.add_action(cartographer)

    return ld
