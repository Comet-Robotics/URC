from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[
                {'use_sim_time': False},  # Set to true if using simulation
                '/path/to/your/nav2_params.yaml'  # Your Nav2 configuration file
            ],
        )
    ])
