from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RealSense Camera Node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[

                {'rgb_camera.color_profile': '640x480x30'},
                {'depth_module.depth_profile': '640x480x30'},
                {"enable_sync": True},
                {"base_frame_id":"base_link"},
                {"align_depth.enable": True},


            ]
        ),
   
        
        # Video Streamer Node
        Node(
            package='video_streamer',
            executable='video_streamer',
            name='video_streamer',
            parameters=[
                {'ip': '100.100.246.41'}
            ]
        ),

        Node(
            package='drivetrain',
            executable='drivetrain_node',
            name='drivetrain',
       
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=['config/cmd_vel.yaml']
        ),
        
        # Basestation Node
        Node(
            package='basestation',
            executable='basestation',
            name='basestation',
        
        ),
    ])
