# Custom Ros Messages (Interfaces)

Many times when working with ros, we seek to make our own message formats either for readibility or necessity.

It is best practice to store those message definitions in a single package.

Refer to this documentation for creating a custom interface: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html

Refer to this documentation fo implementing custom interfaces: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html 

## Troubleshooting

often building this package multiple times will return an error because it does not like to be overwritten (this is ok) if you wish to build a change first run this command from the workspace directory:

``rm -rf build/custom_interfaces/ament_cmake_python/custom_interfaces/custom_interfaces``

to ease with changes simply run

``colcon build --symlink-install``