#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

import xacro


def generate_launch_description():
	
    xacro_file = os.path.join(get_package_share_directory('my_robot_description'), 'urdf/', 'my_robot.urdf.xacro')
    assert os.path.exists(xacro_file), "The xacro doesnt exist in " + str(xacro_file)

    install_dir = get_package_prefix('my_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    pkg_sim_car = get_package_share_directory('my_robot_bringup')


    local_launch_dir = os.path.join(pkg_sim_car, 'launch')


    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = \
            os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = \
            os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    
    world_config = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_sim_car, 'worlds', 'my_floor.world'), ''],
        description='Dae world file')

    spawn_car = Node(
        package="gazebo_ros",
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", "-entity", "my_robot"])
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen")
  
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui")
    

	
    rviz_config_path = os.path.join(pkg_sim_car, 'rviz', 'urdff_config.rviz')
    start_rviz2_cmd = ExecuteProcess(
        cmd=['rviz2', '--display-config', rviz_config_path],
        cwd=[local_launch_dir],
        output='screen')
    

    # mapdir=os.path.join(pkg_sim_car, 'maps', 'navcon1.yaml'),

    # nav2_params_path = os.path.join(
    #     get_package_share_directory('my_robot_bringup'),
    #     'config/',
    #     'nav2_params.yaml')

    # nav_bringup_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(bringup_launch_dir, 'bringup_launch.py')),
    #     launch_arguments={
    #         'use_composition': "1",
    #         # 'slam': "1",
    #         'map': mapdir,
    #         'params_file': nav2_params_path,
    #     }.items(),
    # )

    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )  
    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_sim_car, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ld = LaunchDescription(
        [

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        TimerAction(
            period=0.0,
            actions=[spawn_car]),
        TimerAction(
            period=0.0,
            actions=[robot_state_publisher]),
		TimerAction(
            period=5.0,
            actions=[start_rviz2_cmd]),
        TimerAction(
            period=0.0,
            actions=[joint_state_publisher_gui_node]),
        TimerAction(
            period=0.0,
            actions=[joint_state_publisher_node]),
        TimerAction(
            period=0.0,
            actions=[robot_localization_node]),
        # TimerAction(
        #     period=5.0,
        #     actions=[nav_bringup_cmd]),

     
		# TimerAction(
        #     period=0.0,
        #     actions=[nav_bringup_cmd])
    ])
    ld.add_action(declare_use_sim_time_argument)
    # ld.add_action(declare_map_yaml_cmd)
    # ld.add_action(map_server_cmd)
    # ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(world_config)
    ld.add_action(gazebo)

    #ld.add_action(declare_slam_params_file_cmd)
    #ld.add_action(start_async_slam_toolbox_node)   
    return ld



