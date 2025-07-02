#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    pkg_dir = os.path.dirname(pkg_dir)

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 nav2 parameters file'
    )

    start_nav2_arg = DeclareLaunchArgument(
        'start_nav2',
        default_value='true',
        description='Start Nav2 navigation stack'
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    start_nav2 = LaunchConfiguration('start_nav2')

    # Nav2 parameter substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Nav2 lifecycle nodes
    nav2_lifecycle_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

    # Nav2 Controller Server
    nav2_controller_server = Node(
        condition=IfCondition(start_nav2),
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_nav2_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Nav2 Planner Server
    nav2_planner_server = Node(
        condition=IfCondition(start_nav2),
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Nav2 Behavior Server
    nav2_behavior_server = Node(
        condition=IfCondition(start_nav2),
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_nav2_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Nav2 BT Navigator
    nav2_bt_navigator = Node(
        condition=IfCondition(start_nav2),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_nav2_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Nav2 Lifecycle Manager
    nav2_lifecycle_manager = Node(
        condition=IfCondition(start_nav2),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': nav2_lifecycle_nodes}
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        autostart_arg,
        nav2_params_file_arg,
        start_nav2_arg,

        # Nav2 nodes
        nav2_controller_server,
        nav2_planner_server,
        nav2_behavior_server,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
    ])