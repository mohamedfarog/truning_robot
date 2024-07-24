from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
import launch_ros.descriptions



def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('my_robot_description'), 
                             'urdf' , 'my_robot.urdf.xacro')
    rvize_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                     'rviz', 'urdf_config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2", 
        arguments=['-d', rvize_config_path]
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jsc", "--controller_manger", "/controller_manger"],
    )

    ackermann_steering_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["asc", "--controller_manger", "/controller_manger"],
    )

    


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller_spawner
        
    ])



