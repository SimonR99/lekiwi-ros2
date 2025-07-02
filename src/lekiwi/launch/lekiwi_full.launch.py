#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('lekiwi'), 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 nav2 parameters file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for basic robot visualization'
    )

    use_moveit_rviz_arg = DeclareLaunchArgument(
        'use_moveit_rviz',
        default_value='false',
        description='Launch RViz with MoveIt plugin for motion planning'
    )

    start_nav2_arg = DeclareLaunchArgument(
        'start_nav2',
        default_value='true',
        description='Start Nav2 navigation stack'
    )

    start_moveit_arg = DeclareLaunchArgument(
        'start_moveit',
        default_value='true',
        description='Start MoveIt motion planning'
    )

    zero_pose_arg = DeclareLaunchArgument(
        'zero_pose',
        default_value='false',
        description='Test zero pose after startup'
    )

    start_cameras_arg = DeclareLaunchArgument(
        'start_cameras',
        default_value='true',
        description='Start camera video feeds'
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    autostart = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_moveit_rviz = LaunchConfiguration('use_moveit_rviz')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_moveit = LaunchConfiguration('start_moveit')
    zero_pose = LaunchConfiguration('zero_pose')
    start_cameras = LaunchConfiguration('start_cameras')

    # ===================
    # MODULAR LAUNCH INCLUDES
    # ===================

    # Robot Hardware
    robot_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('lekiwi'), 'launch', 'robot_hardware.launch.py'])
        ]),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'zero_pose': zero_pose,
        }.items(),
    )

    # Navigation Stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('lekiwi'), 'launch', 'navigation.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'nav2_params_file': nav2_params_file,
            'start_nav2': start_nav2,
        }.items(),
        condition=IfCondition(start_nav2),
    )

    # Motion Planning
    motion_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('lekiwi'), 'launch', 'motion_planning.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_moveit': start_moveit,
            'use_moveit_rviz': use_moveit_rviz,
        }.items(),
        condition=IfCondition(start_moveit),
    )

    # Cameras
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('lekiwi'), 'launch', 'cameras.launch.py'])
        ]),
        launch_arguments={
            'start_bottom_camera': 'true',
            'start_wrist_camera': 'false',
        }.items(),
        condition=IfCondition(start_cameras),
    )

    # Visualization
    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('lekiwi'), 'launch', 'visualization.launch.py'])
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    # ===================
    # LAUNCH DESCRIPTION
    # ===================

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_fake_hardware_arg,
        autostart_arg,
        nav2_params_file_arg,
        use_rviz_arg,
        use_moveit_rviz_arg,
        start_nav2_arg,
        start_moveit_arg,
        zero_pose_arg,
        start_cameras_arg,

        # Modular launch includes
        robot_hardware_launch,
        navigation_launch,
        motion_planning_launch,
        cameras_launch,
        visualization_launch,
    ]) 