from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('rtab_package'),
        'config',
    )

    ekf_config = os.path.join(config_path, 'ukf.yaml')
    rtabmap_config = os.path.join(config_path, 'rtabmap_config.yaml')
    navsat_config = os.path.join(config_path, 'navsat_transform.yaml')


    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',  
            'publish_tf': False,
            'wait_for_transform': 0.5,
            'publish_null_when_lost': True,
            'approx_sync': False,
            'use_sim_time': True,
            'guess_from_tf': True,
            'Odom/FillInfoData': 'True',
            'Odom/ResetCountdown': '1',
            'Odom/Holonomic': 'False',
            'Odom/Strategy': '1',
            'rviz': True
        }],
        remappings=[
            ('/rgb/image', '/rgb_image'),
            ('/depth/image', '/depth_image/image_raw'),
            ('/rgb/camera_info', '/rgb_camera_info'),
            ('/depth/camera_info', '/depth_camera_info'),
            ('/odom', '/rgbd_odom')
        ]
    )

    localization = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf",
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
            'publish_null_when_lost': False,
            'wait_for_imu_to_init': True,
            'imu_topic': '/imu',
            'approx_sync': False,
            'use_sim_time': True,
            'publish_tf_odom': False,
            'use_action_for_goal': True,
            'wait_for_transform': 0.5,
            'KP/DetectorStrategy': '6',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ForceOdom3DoF': 'False',
            'RGBD/OptimizeFromGraphEnd': 'True',
            'Grid/MaxGroundHeight': '0.1'
        }],
        remappings=[
            ('/rgb/image', '/rgb_image'),
            ('/depth/image', '/depth_image/image_raw'),
            ('/rgb/camera_info', '/rgb_camera_info'),
            ('/depth/camera_info', '/depth_camera_info'),
            ('/odom', '/odometry/filtered'),
            ('/imu', '/imu')
        ],
    )



    return LaunchDescription([
        rgbd_odometry,
        localization,
        slam
        # rtabmap_launch,
        # navsat_transform
    ])
