from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='occupancy_node',
            output='screen',
            parameters=[{
                'frame_id': 'rover/base_link/camera_front',                     # Set the global frame for the OctoMap
                'resolution': 0.1,                     # Set map resolution, e.g., 0.1 meters
                'sensor_model/max_range': 3.0,          # Maximum sensor range in meters
                'queue_size': 100
            }],
            remappings=[
                ('cloud_in', '/camera/depth/points')     # Replace with your point cloud topic
            ]
        ),
    ])
