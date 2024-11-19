# Rover ROS
This is where we will store all the ROS specific nodes to run on the main computer of the rover currently it is pretty empty.


src/ is where all nodes are kept seperated into directories.


### Depth Image Viewer
Test node to try and pull video data and display on a window to test the realsense.
Take a look at it to see how it works
the only notable source code is in a python file called 
depth-image-viewer/depth-image-node.py 



### Realsense-ros
This is a ros node made by intel that we will use to pull the video feeds from the camera.



### Setup/Dev Enviorement

OS: Ubuntu 22.04

**First Time Setup ONLY**
install ros2 by running
`scripts/install_ros2.sh`
Install realsense
`sudo apt install ros-humble-librealsense2*`
Setup Realsense-ros deps
`rosdep install -i --from-path src --rosdistro humble --skip-keys=librealsense2 -y`




**First Time w/realsense**

`chmod +x scripts/setup_udev_rules.sh`
`./scripts/setup_udev_rules.sh`

**Building Nodes**

URC/rover-ros> `colcon build`
URC/rover-ros> `. ./install/local_setup.sh`


**Running Nodes**

**GPS Node**
run gps node the comm port<br />
`ros2 run gpsx gps_node --ros-args -p "comm_port:=/dev/ttyUSB0" -p "comm_speed:=9600"`

**IMU Node**
run imu node
`ros2 run imu imu_node --ros-args -p "comm_port:=/dev/ttyUSB1"`

Visualizing in rviz:
For first time, install rviz imu plugin
`sudo apt-get install ros-humble-imu-tools`
start base_link and child frame and in rviz select child frame
`ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link child`

**Realsense**
`ros2 run realsense2_camera realsense2_camera_node`
`ros2 run depth_image_viewer depth_image_node`

**Launch Script**
`ros2 launch launch/rover.py`









