# How to Run

### Creating the workspace and setup

1. Recreate Your ROS Workspace  
From what I've seen recreating the rosdep has helped create the workspace
So by starting from scratch you can follow these commands  
``` bash
cd ~/URC/srp_sim/ 
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
```

2. Next you build the srp_sim project by simply running colcon build
#### This is a very important step as everytime you have made a change to the sim_package you must always build it so that ROS and Gazebo can find the new files
``` bash
colcon build
```


3. Finally just source the workspace 
``` bash
. ~/URC/srp_sim/install/setup.sh
```


### Commands to run launch files

To launch any .launch.py file it follows the same format  
"ros2 launch": These are the arugments that makes sure that ros2 is launching it instead of just gazebo  

"sim_package": This is the name of the ros package that contains all the gazebo files from world to models to the lanuch files (without this specifically stated ros will not be able to find the files)

"{specfic file name}.launch.py": selecting whichever launch file you want ot run

#### Launching world
Making sure you are in the ~/URC/srp_sim/ directory and you have built the project successfully, you just run this command  
``` bash
ros2 launch sim_package world.launch.py
```

#### Launching aruco markers
This will spawn the aruco marker into the currently running world and also open rviz (once spawned you may have to move upwards to see the marker)

```bash 
ros2 launch sim_package aruco_marker_launch.launch.py
```

### Troubleshooting Errors

1. Cannot find sim_package  
This could be due to several things however the way ive solved it was removing the previous rosdep init file and rerunning rosdep init in the srp_sim folder  
To remove the rosdep init file run these commands
``` bash
cd ~/etc/ros/rosdep/sources.list.d/
sudo rm -rf 20-default.list
cd ~/URC/srp_sim/
source /opt/ros/humble/setup.bash 
sudo rosdep init
```
And then refollow the steps in setup starting at rosdep update

2. (more to come as it comes up)

