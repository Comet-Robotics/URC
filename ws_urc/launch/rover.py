from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2',
            output='screen',
        ),
        Node(
            package='depth_image_viewer',
            executable='depth_image_node',
            name='image_viewer',
            output='screen',
        ),
  
        # Add more nodes as needed
    ])
