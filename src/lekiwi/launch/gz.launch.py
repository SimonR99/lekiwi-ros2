#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from controller_manager_msgs.srv import SwitchController
import os

def get_robot_description(context, *args, **kwargs):
    dof = LaunchConfiguration('dof').perform(context)
    pkg_share = FindPackageShare('lekiwi').find('lekiwi')
    urdf_path = os.path.join(pkg_share, 'urdf', f'lekiwi_{dof}dof.urdf')
    controller_path = os.path.join(pkg_share, 'config', f'controllers_{dof}dof.yaml')
    
    with open(urdf_path, 'r') as file:
        urdf_content = file.read()
        # Convert package:// to model:// for Gazebo
        replace_str = f'package://lekiwi/models/lekiwi_{dof}dof/meshes'
        with_str = f'model://lekiwi_{dof}dof/meshes'
        gazebo_urdf_content = urdf_content.replace(replace_str, with_str)
        return {
            'robot_description': ParameterValue(urdf_content, value_type=str),
            'gazebo_description': ParameterValue(gazebo_urdf_content, value_type=str),
            'controller_path': controller_path
        }

def generate_launch_description():
    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value='5',
        description='DOF configuration - either 5 or 7'
    )

    pkg_share = FindPackageShare('lekiwi').find('lekiwi')
    model_path = os.path.join(os.path.dirname(os.path.dirname(pkg_share)), 'models')

    # Set Gazebo model path
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += f":{model_path}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = model_path

    def launch_setup(context, *args, **kwargs):
        descriptions = get_robot_description(context)
        
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_model',
            arguments=[
                '-string', descriptions['gazebo_description'].value,
                '-name', 'lekiwi',
                '-allow_renaming', 'true',
                '-x', '0',
                '-y', '0',
                '-z', '0'
            ],
            output='screen'
        )

        joint_state_broadcaster_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
            output='screen'
        )

        joint_trajectory_controller_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_trajectory_controller'],
            output='screen'
        )

        gripper_controller_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'gripper_controller'],
            output='screen'
        )

        nodes = [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': descriptions['robot_description']}]
            ),
            
            ExecuteProcess(
                cmd=['gz', 'sim', '-r', 'empty.sdf'],
                output='screen',
                additional_env={'GZ_SIM_RESOURCE_PATH': os.environ['GZ_SIM_RESOURCE_PATH']}
            ),

            # Bridge between ROS2 and Gazebo
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge',
                parameters=[{
                    'qos_overrides./tf_static.publisher.durability': 'transient_local',
                }],
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                    '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                ],
            ),

            spawn_robot,

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_broadcaster_spawner]
                )
            ),

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[joint_trajectory_controller_spawner]
                )
            ),

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_trajectory_controller_spawner,
                    on_exit=[gripper_controller_spawner]
                )
            )
        ]
        return nodes

    return LaunchDescription([
        dof_arg,
        OpaqueFunction(function=launch_setup)
    ])