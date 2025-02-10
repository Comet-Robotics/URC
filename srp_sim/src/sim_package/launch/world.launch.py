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
    pkg_share = get_package_share_directory('sim_package')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf_config.rviz')
    models_dir = os.path.join(pkg_share, 'models')
    rover_model = os.path.join(models_dir, 'rover/model.xacro')
 
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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', rover_model])}]
    )

    spawn = Node( package='ros_gz_sim', executable='create', arguments=[ '-name', 'rover', '-topic', 'robot_description', ], output='screen', ) 


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', rover_model])}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=models_dir,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        set_gazebo_model_path, 
        world_client,
        bridge,
        spawn,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])