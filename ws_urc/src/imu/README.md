# IMU 
The Inertial Measurement Unit, or IMU, is a sensor to read and publish orientation, angular velocity, and linear acceleration data. This package is an implementation of a ROS 2 node and communicates with the IMU sensor over a serial connection.

## Components

### 1. `imu_node.py`
This is the main script of the package and contains the `IMU` class, which is a ROS 2 node. The key functionalities include:
-> A Serial Communication, which establishes a serial connection with the IMU sensor using a self-declared parameters such as `comm_port` and `baud_rate`.
-> Data Processing, which reads IMU data from the serial port, parses it, and converts it into ROS message formats.
-> Publishing, which helps send messages with regards to data:
  -> `/imu`, that contains `sensor_msgs/Imu`, that publishes messages with angular velocity, linear acceleration, and orientation data.
  -> `/robot_orientation`, that contains `geometry_msgs/Vector3`, that publishes messages with roll, pitch, and yaw values.
-> Error Handling, that monitors the serial connection and shuts it down after a certain number of consecutive errors, which can be modified.

### 2. `setup.py`
Defines the IMU package metadata and entry points. The `imu_node` script is registered as a console script, making it an executable ROS 2 node.

### 3. `test/`
Contains unit tests for the package, including:
-> `test_copyright.py`, which ensures that we are compliant with copyright.
-> `test_flake8.py`, which checks the code style using Flake8.
-> `test_pep257.py`, which verifies accordance with PEP 257 docstring conventions.

## How It Works
1. Initialization: The `IMU` node initializes by declaring parameters for the serial port and baud rate. It then sets up publishers for IMU data and robot orientation.
2. Serial Communication: The node establishes a serial connection to the IMU sensor and starts a timer to periodically read data.
3. Data Parsing: The IMU data is read from the serial port, parsed, and converted into ROS message formats.
4. Publishing: The processed data is then published to the `/imu` and `/robot_orientation`.
5. Error Handling: If the node faces consistent errors while reading the data, it shuts down the serial connection to prevent further issues.

## Usage
To run the IMU node, use the following command:
```bash:
ros2 run imu imu_node
```

Make sure that the `comm_port` parameter is correctly set to the serial port of the IMU sensor.