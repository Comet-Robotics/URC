from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions

def generate_launch_description():
    package_name = 'robot_simulation'
    package_share_directory = get_package_share_directory(package_name)
    ekf_config = os.path.join(package_share_directory, 'config', 'ekf.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            "output_final_position", default_value="false"
        ),
        DeclareLaunchArgument(
            "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true"
        ),
        DeclareLaunchArgument(
            "ekf_config", default_value=ekf_config
        ),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[LaunchConfiguration("ekf_config"), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[LaunchConfiguration("ekf_config"), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[("odometry/filtered", "odometry/global")],
        ),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[LaunchConfiguration("ekf_config"), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[
                ("imu/data", "imu/data"),
                ("gps/fix", "gps/fix"),
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),
                ("odometry/filtered", "odometry/global"),
            ],
        ),
    ])