
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get needed package directories
    pkg_gazebo = get_package_share_directory('ros_gz_sim')
    pkg_robot = get_package_share_directory('basic_sim')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the path to the robot URDF file
    urdf_file = PathJoinSubstitution([
        pkg_robot, 'description', 'robot.urdf.xacro'
    ])

    # Process the URDF file with xacro
    robot_description = Command(['xacro ', urdf_file])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch Gazebo (Gazebo Sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_gazebo, '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot'
        ],
        parameters=[{'robot_description': robot_description}],
    )

    # Startup the gz bridge for cmd_vel
    bridge_param_file = PathJoinSubstitution([
        pkg_robot, 'config', 'gz_bridge.yaml'
    ])
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_param_file
        }]
    )

    # Launch
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])

