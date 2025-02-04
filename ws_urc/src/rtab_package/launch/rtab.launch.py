from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('rtab_package'),
        'config',
    )

    ekf_config = os.path.join(config_path, 'ekf.yaml')
    navsat_config = os.path.join(config_path, 'navsat_transform.yaml')

    # Define paths for launch files
    rtabmap_launch_path = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    )


    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[{
            'frame_id': 'camera_link_optical',
            'odom_frame_id': 'odom',  
            'publish_tf': True,
            'wait_for_transform': 0.1,
            'publish_null_when_lost': False,
            'wait_for_imu_to_init': True,
            'imu_topic': '/imu',
            'approx_sync': False,
            'use_sim_time': True
        }],
        remappings=[
            ('/rgb/image', '/image'),
            ('/depth/image', '/depth_image'),
            ('/rgb/camera_info', '/camera_info')
        ]
    )

    localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf",
        output="screen",
        parameters=[ekf_config],
    )

    # navsat_transform = Node(
    #     package='robot_localization',
    #     executable='navsat_transform_node',
    #     name='navsat_transform',
    #     output='screen',
    #     parameters=[navsat_config],
    #     remappings=[('/fix', '/navsat'),  
    #                 ('/imu', '/imu')]  
    # )

    slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        arguments=['--delete_db_on_start'],
        parameters=[{
            'subscribe_depth': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',  
            'publish_tf': True,
            'publish_null_when_lost': True,
            'wait_for_imu_to_init': True,
            'imu_topic': '/imu',
            'approx_sync': False,
            'use_sim_time': True,
            'use_action_for_goal': True,
        }],
        remappings=[
            ('/rgb/image', '/image'),
            ('/depth/image', '/depth_image'),
            ('/rgb/camera_info', '/camera_info'),
            ('/odom', 'odometry/filtered'),
            ('/imu', '/imu')
        ],
    )

    return LaunchDescription([
        rgbd_odometry,
        localization,
        slam
        # navsat_transform
    ])
