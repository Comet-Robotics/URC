# Sim Package

## What is Gazebo
- Gazebo is a simulation software that mimics the physics of the real world and allows for seamless testing when testing with the physical thing is unsafe or impractical.

- Gazebo can simulate environments, wiggle, cameras, and most any sensors

## How does Gazebo
- Gazebo is its own standalone open source software, it connects through a bridge to ros which basically tells ros what the world is doing to the robot. It will send back camera info, odometry (feedback on how much the wheel moved to compare against a request), gps, etc.

- To simulate the world and the bot, Gazebo uses one of 3 main types of files all in XML (a type of markup language, think HTML):
1. **SDF** (Simulation Description Format) this one is the most basic with little features
2. **URDF** (Universal Robot Description Format) this is the most commonly used one and more modern
3. **xacro** is a macro type file that ends with .xacro and converts to a URDF after compiling, but gives tools like constants/variables, math, and method like macros. for example instead of manually typing each objects moment of inertia, you call a premade method with 

- It is recommended to use Xacro files for easy editing and modern development. However all these file types have similar syntax with minor variations,

- These XML files describe the geometry, collision shapes, inertial properties, visual appearance, and mass of objects. 

# How to Run

### Installing Gazebo Harmonic
``` bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
```

``` bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```


``` bash
export ROS_DISTRO=jazzy
sudo apt install ros-$ROS_DISTRO-ros-gz-sim ros-$ROS_DISTRO-ros-gz-bridge

```



### Creating the workspace and setup

1. Recreate Your ROS Workspace  
From what I've seen recreating the rosdep has helped create the workspace
So by starting from scratch you can follow these commands  
``` bash
cd ~/URC/srp_sim/ 
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro $ROS_DISTRO
```

2. Next you build the srp_sim project by simply running colcon build
#### This is a very important step as everytime you have made a change to the sim_package you must always build it so that ROS and Gazebo can find the new files
``` bash
colcon build --symlink-install
```


3. Finally just source the workspace 
``` bash
. ~/URC/srp_sim/install/setup.sh
```


### Commands to run launch files

To launch any .launch.py file it follows the same format  
"ros2 launch": These are the arugments that makes sure that ros2 is launching it instead of just gazebo  

"sim_package": This is the name of the ros package that contains all the gazebo files from world to models to the lanuch files (without this specifically stated ros will not be able to find the files)

"{specific file name}.launch.py": Specifies which launch file to run

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
cd /etc/ros/rosdep/sources.list.d/
sudo rm -rf 20-default.list
cd ~/URC/srp_sim/
source /opt/ros/$ROS_DISTRO/setup.bash 
sudo rosdep init
```
And then refollow the steps in setup starting at rosdep update

2. (more to come as it comes up)

