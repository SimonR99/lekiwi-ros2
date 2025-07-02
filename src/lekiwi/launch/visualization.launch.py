#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for basic robot visualization'
    )

    # Launch configurations
    use_rviz = LaunchConfiguration('use_rviz')

    # Basic RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('lekiwi'), 'config', 'urdf.rviz'])]
    )

    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,

        # RViz nodes
        rviz_node,
    ])