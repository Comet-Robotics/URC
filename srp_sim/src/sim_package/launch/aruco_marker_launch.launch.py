# aruco_marker_spawn.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
import xacro
import launch.utilities
import launch_ros.descriptions as descriptions
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # Define the path to the ArUco marker SDF model
    pkg_share = get_package_share_directory('sim_package')
    aruco_model_path = os.path.join(
        pkg_share,
        'models', '4x4_50-5', 'aruco_marker.xacro'
    )
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Define the world file to load into Gazebo
    world_file_path = os.path.join(
        pkg_share,
        'worlds', 'world.sdf'
    )
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'aruco_marker_config.rviz')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
           'gz_args': ['-r -s -v4', world_file_path], 'on_exit_shutdown':'true'
        }.items(),
    )
    state_publisher_config = xacro.process_file(
        aruco_model_path,
        mappings={
            'model_file' : '4X4_50-5.dae'
        },
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=['aruco_4X4_50_', LaunchConfiguration('marker')],
        parameters=[
            # Use xacro to correctly spawn the correct lidar
            {'robot_description': state_publisher_config.toxml()
            }]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    aruco_spawn = Node(
        package='ros_gz_sim', 
        executable='create',
        namespace=['aruco_4X4_50_', LaunchConfiguration('marker')],
        arguments=['-entity', '4X4_50', 
                    '-name', ['4X4_50-', LaunchConfiguration('marker')],
                    '-topic', 'robot_description',
                        '-x', LaunchConfiguration('X'),
                        '-y', LaunchConfiguration('Y'),
                        '-z', '10',
                        '-Y', '0.0'],
                        output='screen')

    ld = launch.LaunchDescription([
        # Launch Gazebo with the specified world
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),

        # Is rViz wanted
        launch.actions.DeclareLaunchArgument(name='rviz', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),

        # What is the model file path
        launch.actions.DeclareLaunchArgument(name='model', default_value=aruco_model_path,
                                            description='Absolute path to robot urdf file'),

        # What is the rviz config file path
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        # What is the marker to be spawned
        launch.actions.DeclareLaunchArgument(name='marker', default_value='5',
                                            description='The Marker Number to be spawned'),

        # What is the marker X coordinate
        launch.actions.DeclareLaunchArgument(name='X', default_value='0.0',
                                            description='The X position for the marker to be spawned'),

        # What is the marker Y coordinate
        launch.actions.DeclareLaunchArgument(name='Y', default_value='0.0',
                                            description='The Y position for the marker to be spawned')
        ,

        # Spawn the ArUco marker model into Gazebo
        
    ])
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(aruco_spawn)

    return ld

