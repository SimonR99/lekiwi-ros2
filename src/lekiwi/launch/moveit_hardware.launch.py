from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt Configuration
    moveit_config = MoveItConfigsBuilder("lekiwi", package_name="lekiwi").to_moveit_configs()

    # Get URDF via xacro with real hardware
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution(
                    [FindPackageShare('lekiwi'), 'config', 'lekiwi.urdf.xacro']
                ),
                ' ',
                'use_fake_hardware:=false'
            ]
        ),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ros2_control using real hardware
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

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn arm controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Spawn gripper controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # MoveIt move_group node
    move_group_node = Node(
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
            {"use_sim_time": False},
        ],
    )

    # RViz with MoveIt
    rviz_node = Node(
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

    # Event handlers to sequence the startup
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_gripper_controller_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # Delay MoveIt to start after controllers are loaded
    delay_move_group_after_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    # Delay RViz to start after MoveIt
    delay_rviz_after_move_group = TimerAction(
        period=5.0,  # Wait 5 seconds after launch starts
        actions=[rviz_node]
    )

    nodes = [
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_gripper_controller_spawner_after_robot_controller_spawner,
        delay_move_group_after_gripper_controller,
        delay_rviz_after_move_group,
    ]

    return LaunchDescription(nodes) 