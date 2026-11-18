# TODO: Update this file to use sensors and enable the ros_gz_bridge.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def generate_launch_description():

    # FindPackageShare is used to find the path to a package's share directory. This is where files related to that packages will be stored (defined in setup.py).
    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    world_sdf_path = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "examples", "level_example.sdf"]
    )

    # This gets the URDF file for the robot from robot_description. It then converts it from xacro to a URDF string.
    # Gazebo cannot read xacro files, so they must be converted to URDF before being passed to Gazebo.
    # urdf_string is a description string of the robot that Gazebo can read.
    robot_base = 'srp'  # Change robot base here if needed
    urdf_string = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare("robot_description"),
            "urdf",
            f"{robot_base}.urdf.xacro"
        ])
    ])

    # This node will spawn the rover model. You can also define nodes directly in the return statement, 
    # but defining it here is required to use the event handler to prevent a race condition.
    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', urdf_string, 
            '-name', 'srp_robot', 
            '-x', '0',
            '-y', '0',
            '-z', '0.5',
            '-Y', '0',
        ]
    )

    # This command will set the rover as a performer, again could be defined directly in the return statement,
    # but defining it here is required to use the event handler to prevent a race condition.
    set_performer = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/world/levels_world/level/set_performer',
            '--reqtype', 'gz.msgs.StringMsg',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req','data: "srp_robot"',
        ],
        output='screen'
    )

    # This event handler will prevent the set_performer command from running until after the rover has been spawned
    register_performer_on_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_rover, on_exit=[set_performer],
        )
    )

    # The LaunchDescription is a list of actions that will be executed when the launch file is run.
    # This is not neccessarily sequential, and some actions may be executed in parallel, hence why we use register_performer_on_spawn to ensure the set_performer command is not run until after the rover has been spawned.
    return LaunchDescription([

        DeclareLaunchArgument(
            name='gui', 
            default_value='true',
            description='Enable Gazebo Client'
        ),

        # Gazebo server
        # IncludeLaunchDescription is used to include another launch file.
        # This launch file starts Gazebo in Server mode, and with levels enabled
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={'gz_args': ['-r -s -v4 ', world_sdf_path, ' --levels']}.items(),
        ),

        # Gazebo client
        # This launch file starts Gazebo in Client mode, which is the GUI. While not required for simulation, it lets us see into the world.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={'gz_args': [' -g']}.items()
        ),


        # Pass in the spawn_rover node we defiend above to run it.
        spawn_rover,
        # set_performer doesn't need to be passed in here because the event handler will run it itself.
        register_performer_on_spawn
    ])
    