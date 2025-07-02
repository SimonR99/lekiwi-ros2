#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    start_moveit_arg = DeclareLaunchArgument(
        'start_moveit',
        default_value='true',
        description='Start MoveIt motion planning'
    )

    use_moveit_rviz_arg = DeclareLaunchArgument(
        'use_moveit_rviz',
        default_value='false',
        description='Launch RViz with MoveIt plugin for motion planning'
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_moveit = LaunchConfiguration('start_moveit')
    use_moveit_rviz = LaunchConfiguration('use_moveit_rviz')

    # MoveIt Configuration
    moveit_config = MoveItConfigsBuilder("lekiwi", package_name="lekiwi").to_moveit_configs()

    # MoveIt move_group node
    move_group_node = Node(
        condition=IfCondition(start_moveit),
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.moveit_cpp,
            moveit_config.planning_scene_monitor,
            moveit_config.sensors_3d,
            {"use_sim_time": use_sim_time},
        ],
    )

    # MoveIt RViz
    moveit_rviz_node = Node(
        condition=IfCondition(use_moveit_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare('lekiwi'), 'config', 'moveit.rviz'])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Delay MoveIt RViz to start after move_group
    delay_moveit_rviz_after_move_group = TimerAction(
        period=3.0,  # Wait 3 seconds after move_group starts
        actions=[moveit_rviz_node]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        start_moveit_arg,
        use_moveit_rviz_arg,

        # MoveIt nodes
        move_group_node,
        delay_moveit_rviz_after_move_group,
    ])