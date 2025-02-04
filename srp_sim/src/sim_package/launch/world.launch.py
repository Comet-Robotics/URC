# aruco_marker_spawn.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  AppendEnvironmentVariable, ExecuteProcess,SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
import xacro
import launch.utilities
import launch_ros.descriptions as descriptions
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, TextSubstitution
import logging


def generate_launch_description():
    # Define the path to the ArUco marker SDF model
    pkg_share = get_package_share_directory('sim_package')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf_config.rviz')

    models_dir = os.path.join(pkg_share, 'models')
 
    print(models_dir)
    # Set GAZEBO_MODEL_PATH environment variable
    set_gazebo_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([models_dir, os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    )
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Define the world file to load into Gazebo
    world_file_path = os.path.join(
        pkg_share, 'worlds', 'world.sdf'
    )
    

    world_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
           'gz_args': ['-r -s -v4', world_file_path], 'on_exit_shutdown':'true'
        }.items(),
    )

    world_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([pkg_share,'worlds','world.sdf'])}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
        }],
        output='screen'
    )

    sdf = os.path.join(
    pkg_share,
    'models', 'rover', 'model.sdf')

    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)

    robot_state_publisher =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'robot_description': doc.toxml()}])
    

    camera_front_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_front_to_camera_link',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '1.57079632679', '3.14159265359', '1.57079632679', 'rover/camera_link', 'rover/camera_link/camera_front']
    )

    worldflip = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_tove',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '-1.57079632679', '0.0', 'odom', 'base_link']
    )
    
    return LaunchDescription([
               set_gazebo_model_path,
        # worldflip,
        world_client,
        bridge,
        robot_state_publisher,
        camera_front_to_camera_link
    ])


    
    

