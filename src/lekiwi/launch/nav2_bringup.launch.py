#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    pkg_dir = os.path.dirname(pkg_dir)
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Nav2 lifecycle nodes that need to be managed
    lifecycle_nodes = ['controller_server',
                      'smoother_server', 
                      'planner_server',
                      'behavior_server',
                      'bt_navigator',
                      'waypoint_follower',
                      'velocity_smoother']

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    bringup_cmd_group = GroupAction([
        Node(
            condition=IfCondition(PythonExpression([
                'not ', use_composition])),
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        Node(
            condition=IfCondition(PythonExpression([
                'not ', use_composition])),
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        Node(
            condition=IfCondition(PythonExpression([
                'not ', use_composition])),
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        Node(
            condition=IfCondition(PythonExpression([
                'not ', use_composition])),
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        Node(
            condition=IfCondition(PythonExpression([
                'not ', use_composition])),
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        Node(
            condition=IfCondition(PythonExpression([
                'not ', use_composition])),
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        Node(
            condition=IfCondition(PythonExpression([
                'not ', use_composition])),
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/cmd_vel', '/cmd_vel_nav'),
                       ('/cmd_vel_smoothed', '/cmd_vel'),
                       ('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        Node(
            condition=IfCondition(PythonExpression([
                'not ', use_composition])),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                       {'autostart': autostart},
                       {'node_names': lifecycle_nodes}]),

        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_controller',
                    plugin='nav2_controller::ControllerServer',
                    name='controller_server',
                    parameters=[configured_params],
                    remappings=[('/tf', 'tf'),
                               ('/tf_static', 'tf_static')]),
                ComposableNode(
                    package='nav2_smoother',
                    plugin='nav2_smoother::SmootherServer',
                    name='smoother_server',
                    parameters=[configured_params],
                    remappings=[('/tf', 'tf'),
                               ('/tf_static', 'tf_static')]),
                ComposableNode(
                    package='nav2_planner',
                    plugin='nav2_planner::PlannerServer',
                    name='planner_server',
                    parameters=[configured_params],
                    remappings=[('/tf', 'tf'),
                               ('/tf_static', 'tf_static')]),
                ComposableNode(
                    package='nav2_behaviors',
                    plugin='behavior_server::BehaviorServer',
                    name='behavior_server',
                    parameters=[configured_params],
                    remappings=[('/tf', 'tf'),
                               ('/tf_static', 'tf_static')]),
                ComposableNode(
                    package='nav2_bt_navigator',
                    plugin='nav2_bt_navigator::BtNavigator',
                    name='bt_navigator',
                    parameters=[configured_params],
                    remappings=[('/tf', 'tf'),
                               ('/tf_static', 'tf_static')]),
                ComposableNode(
                    package='nav2_waypoint_follower',
                    plugin='nav2_waypoint_follower::WaypointFollower',
                    name='waypoint_follower',
                    parameters=[configured_params],
                    remappings=[('/tf', 'tf'),
                               ('/tf_static', 'tf_static')]),
                ComposableNode(
                    package='nav2_velocity_smoother',
                    plugin='nav2_velocity_smoother::VelocitySmoother',
                    name='velocity_smoother',
                    parameters=[configured_params],
                    remappings=[('/cmd_vel', '/cmd_vel_nav'),
                               ('/cmd_vel_smoothed', '/cmd_vel'),
                               ('/tf', 'tf'),
                               ('/tf_static', 'tf_static')]),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    parameters=[{'use_sim_time': use_sim_time,
                                'autostart': autostart,
                                'node_names': lifecycle_nodes}]),
            ],
        )
    ])

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(bringup_cmd_group)

    return ld