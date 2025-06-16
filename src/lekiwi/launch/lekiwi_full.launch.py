#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
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
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
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

    # MoveIt Configuration
    moveit_config = MoveItConfigsBuilder("lekiwi", package_name="lekiwi").to_moveit_configs()

    # Get URDF via xacro
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

    # ===================
    # HARDWARE NODES
    # ===================

    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Controller Manager
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

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Arm Controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Wheel Controller
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_wheel_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Holonomic Controller
    holonomic_controller_node = Node(
        package='lekiwi_hardware',
        executable='holonomic_controller.py',
        name='holonomic_controller',
        parameters=[{
            'wheel_radius': 0.05,
            'base_radius': 0.125,
            'max_wheel_velocity': 3.0,
            'cmd_timeout': 0.2,
            'safety_check_rate': 10.0,
        }],
        output='screen',
    )

    # Odometry Publisher
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

    # Zero Pose Test Node
    zero_pose_node = Node(
        condition=IfCondition(zero_pose),
        package='lekiwi_hardware',
        executable='zero_pose.py',
        name='zero_pose_test',
    )

    # ===================
    # NAV2 NODES
    # ===================

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

    # ===================
    # MOVEIT NODES
    # ===================

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

    # ===================
    # RVIZ NODES
    # ===================

    # Basic RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('lekiwi'), 'config', 'urdf.rviz'])]
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

    # ===================
    # EVENT HANDLERS (SEQUENCING)
    # ===================

    # Sequence: joint_state_broadcaster -> robot_controller
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Sequence: robot_controller -> gripper_controller
    delay_gripper_controller_after_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # Sequence: gripper_controller -> wheel_controller
    delay_wheel_controller_after_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[wheel_controller_spawner],
        )
    )

    # Sequence: wheel_controller -> moveit (if enabled)
    delay_moveit_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wheel_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    # Delay MoveIt RViz to start after move_group
    delay_moveit_rviz_after_move_group = TimerAction(
        period=3.0,  # Wait 3 seconds after move_group starts
        actions=[moveit_rviz_node]
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

        # Hardware nodes
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_robot_controller_after_joint_state,
        delay_gripper_controller_after_robot_controller,
        delay_wheel_controller_after_gripper_controller,
        holonomic_controller_node,
        odometry_publisher_node,
        zero_pose_node,

        # Nav2 nodes
        nav2_controller_server,
        nav2_planner_server,
        nav2_behavior_server,
        nav2_bt_navigator,
        nav2_lifecycle_manager,

        # MoveIt nodes  
        delay_moveit_after_controllers,

        # RViz nodes
        rviz_node,
        delay_moveit_rviz_after_move_group,
    ]) 