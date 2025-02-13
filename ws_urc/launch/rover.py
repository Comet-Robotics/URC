from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ip = "100.117.177.44"
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
        Node(
            package="gpsx",
            executable='gps_node',
            name='gps_node',
            parameters=[
                {"comm_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_186b31900bb6ed119254dc0ea8669f5d-if00-port0"},
                {"comm_speed": 9600}
            ]
        ),
        Node(
            package='imu',
            executable='imu_node',
            name='imu_node'

        ),
    
   
        
        # Video Streamer Node
        Node(
            package='video_streamer',
            executable='video_streamer',
            name='video_streamer',
            parameters=[
                {'ip': ip}
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
            executable='basestation_node',
            name='basestation',
            parameters=[
                {'ip': ip}
            ]
        
        ),
])
