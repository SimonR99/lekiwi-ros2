from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # URDF file argument
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='lekiwi_complete.urdf',
        description='URDF file to load'
    )

    # Get URDF file path
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='cat'), ' ',
                PathJoinSubstitution(
                    [FindPackageShare('lekiwi'), 'urdf', LaunchConfiguration('urdf_file')]
                )
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

    # # Joint State Publisher (for testing joint movement)
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    # )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('lekiwi'), 'config', 'urdf.rviz'])]
    )

    return LaunchDescription([
        urdf_file_arg,
        robot_state_pub_node,
        # joint_state_publisher_node,
        rviz_node
    ]) 