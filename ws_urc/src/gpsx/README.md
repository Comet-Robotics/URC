# GPSX
This node helps read and extract data from GPS by first establishing a serial port connection based on the baud rate, which is the speed of the connection and stored in the variable `serial_portp`. It then reads raw GPS messages from these serial ports, parses them into readable file format, and publishes the data as ROS messages.

## Packages Used

### `rclcpp`
This is the core ROS 2 client library used to create the `GPSPublisher` node, which is the used for declaring and retrieving parameters of `comm_port` and `comm_speed`, then set up publishers, services, and timers, and finally, log messages for debugging and status updates.

### `sensor_msgs/msg/nav_sat_fix.hpp` and `sensor_msgs/msg/nav_sat_status.hpp`
These are the standard ROS 2 message types for GPS data:
- `NavSatFix`: Fixes the current GPS position, including latitude, longitude, and altitude.
- `NavSatStatus`: Provides the status of the GPS receiver.

### `gpsx/msg/gpsx.hpp`
This is a custom message type defined in the `gpsx` package. It is used to publish parsed GPS data, including:
- Latitude, longitude, and altitude.
- Number of satellites in view.
- Ground speed and course information.

### `gpsx/srv/get_sat_list.hpp`
This is a custom service type defined in the `gpsx` package. It is used to:
- Handle requests for satellite information.
- Provide details such as satellite IDs, types, elevations and signal-to-noise ratios (SNR).

### Standard C++ Libraries used
- **`<fstream>`** is used as part of serial port communication to read GPS data from a file.
- **`<string>`**: It handles string manipulation to parse GPS messages.
- **`<vector>`**: It is used to update and store a list of elements dynamically. In this case, it is used to store changes for satellite data and parsed messages.
- **`<termios.h>`**: It is used to configure the Terminal I/O devices, which is used for serial port settings.

### 6. ROS 2 Timer and Publisher
- A timer is used to periodically read and process GPS data.
- The processed data is published to the `gpsx` topic using the `gpsx::msg::Gpsx` message type.

### 7. ROS 2 Service
- The `get_sat_list` service is implemented to respond to requests for satellite data. It uses the `gpsx::srv::GetSatList` service type.

## Usage
To run the GPSX node, use the following command:
```bash
ros2 run gpsx gps_node
```

Make sure that the `comm_port` and `comm_speed` parameters are correctly set to match the serial port and baud rate of the GPS device.
