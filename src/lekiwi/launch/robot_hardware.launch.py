#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware'
    )

    zero_pose_arg = DeclareLaunchArgument(
        'zero_pose',
        default_value='false',
        description='Test zero pose after startup'
    )

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    zero_pose = LaunchConfiguration('zero_pose')

    # Generate robot description from xacro
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution(
                    [FindPackageShare('lekiwi'), 'config', 'lekiwi.urdf.xacro']
                ),
                ' ',
                'use_fake_hardware:=', use_fake_hardware
            ]
        ),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare('lekiwi'), 'config', 'controllers.yaml']
            ),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_wheel_controller", "-c", "/controller_manager"],
        output="screen",
    )

    holonomic_controller_node = Node(
        package='lekiwi_hardware',
        executable='holonomic_controller.py',
        name='holonomic_controller',
        parameters=[{
            'wheel_radius': 0.05,
            'base_radius': 0.125,
            'max_wheel_velocity': 3.0,
            'cmd_timeout': 0.2,
            'safety_check_rate': 50.0,
        }],
        output='screen',
    )

    odometry_publisher_node = Node(
        package='lekiwi_hardware',
        executable='odometry_publisher.py',
        name='odometry_publisher',
        parameters=[{
            'wheel_radius': 0.05,
            'base_radius': 0.125,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'publish_tf': True,
        }],
        output='screen',
    )

    zero_pose_node = Node(
        condition=IfCondition(zero_pose),
        package='lekiwi_hardware',
        executable='zero_pose.py',
        name='zero_pose_test',
    )

    # Controller sequencing - ensure proper startup order
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_gripper_controller_after_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    delay_wheel_controller_after_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[wheel_controller_spawner],
        )
    )

    return LaunchDescription([
        use_fake_hardware_arg,
        zero_pose_arg,
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_robot_controller_after_joint_state,
        delay_gripper_controller_after_robot_controller,
        delay_wheel_controller_after_gripper_controller,
        holonomic_controller_node,
        odometry_publisher_node,
        zero_pose_node,
    ])