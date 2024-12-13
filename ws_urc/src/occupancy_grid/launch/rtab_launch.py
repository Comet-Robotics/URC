import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    ld = LaunchDescription()
    launch_file_path_rs = os.path.join(get_package_share_directory('scatcat_bringup'), 'launch')
    # Create the launch configuration variables
    enable_gyro = LaunchConfiguration('enable_gyro')
    enable_accel = LaunchConfiguration('enable_accel')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    enable_sync = LaunchConfiguration('enable_sync')
    # 2 Affecter une valeur aux variables
    declare_enable_gyro = DeclareLaunchArgument(
    'enable_gyro', default_value='true',
    description='Automatically force ')
    declare_enable_accel = DeclareLaunchArgument(
    'enable_accel', default_value='true',
    description='Automatically force ')
    declare_unite_imu_method = DeclareLaunchArgument(
    'unite_imu_method', default_value='1',
    description='Automatically force ')
    declare_enable_sync = DeclareLaunchArgument(
    'enable_sync', default_value='false',
    description='Automatically force ')
    launch_file_path_rtabmap = os.path.join(get_package_share_directory('rtabmap_ros'), 'launch')

    rviz_config_file = PathJoinSubstitution(
    [FindPackageShare("scatcat_bringup"), "rviz", "rtabmap.rviz"])
    start_rs = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    os.path.join(launch_file_path_rs, 'rs_launch.py')),
    launch_arguments={'enable_gyro': enable_gyro,
    'enable_accel': enable_accel,
    'unite_imu_method': unite_imu_method,
    'enable_sync': enable_sync }.items()
    )

    start_rtabmap = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    os.path.join(launch_file_path_rtabmap, 'realsense_d435i_color.launch.py')))
    visu_rviz = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    # output="log",
    output="screen",
    arguments=["-d", rviz_config_file])

    ld.add_action(declare_enable_gyro)
    ld.add_action(declare_enable_accel)
    ld.add_action(declare_unite_imu_method)
    ld.add_action(declare_enable_sync)
    ld.add_action(start_rs)
    ld.add_action(start_rtabmap)
    ld.add_action(visu_rviz)
    return ld