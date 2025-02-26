# UTD University Rover Challenge

This is where all of our code and most of the project management will be base at.

## Overview

Discussions: Where all research topics will go and where you will log your ideas, progress, and questions at. I will also post research materials there too.

Issues: Once research is done we will create an issue for a topic and this is where you will contribute your code too and discuss issues that are arriving in the implementation.

## Instructions
- Start up the simulation and press play. 


- In another terminal, cd to rtab_package (such a dumb name we can definitely change this) which is in the ws_urc src folder, colcon build, source the install. 
- To run, "ros2 launch rtab_package rtab.launch.py". 
  - This node takes care of all transforms needed for the Nav2 stack.
  - rgbd odometry creates an odom -> base_link odometry message
  - ukf robot_localization fuses the IMU odometry with the visual odometry and publishes the odom -> base_link transformation
  - rgbd-slam publishes the map -> odom frame
  - RTAB-MAP frame to frame model
 
- Then, cd to navigation, which is also in the ws_urc src folder. Colcon build, source the install.
  - Run with "ros2 launch navigation nav.launch.py"
  - This initializes the nav2 stack, nothing outputs after initialization unless a goal is given
 
- Start rviz2
  - Set the global frame to "map"
  - include "Robot Description" topic to see the robot (the wheels dont move with it even though the transforms are being published, don't know what's going on there)
  - Include "odometry" topic, set ros2 topic to "odometry/filtered" to see the fused odometry
  - Include "/map" topic (this doesn't show up until nav2 is activated)
  - Include "goal pose" to see the given goal
  - Go to panels, click on navigation2. It should say the navigation is active. 
  - To send actions to the robot, in rviz, select 2d goal pose and place it on the map. Click Start Waypoint Navigation in the nav2 panel to start navigation

Install git 

git clone <github_uri>

