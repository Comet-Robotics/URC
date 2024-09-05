ROS_DISTRO=humble

sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
