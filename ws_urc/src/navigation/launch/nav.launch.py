from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define directories and file paths for configuration files
    config_dir = os.path.join(
        get_package_share_directory('navigation'), 'config'
    )

    nav2_params_path = os.path.join(config_dir, 'nav2_params.yaml')


    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )


    nav2_launch = IncludeLaunchDescription(
        nav2_launch_path,
        launch_arguments={
                'params_file': nav2_params_path,
                'use_sim_time': "True",
                # 'log_level': 'debug'
        }.items(),
    )

    return LaunchDescription([
        nav2_launch
    ])
