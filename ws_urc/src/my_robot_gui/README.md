# my_robot_gui

This package contains a simple gui which publishes joint rotation values to a controller topic

## How to run:

#### Install Dependencies:
Inside the `ws_urc` folder, run:
``` bash
rosdep install --from-path my_robot_gui --ignore-src -y
```

#### Build the package:
``` bash
colcon build --packages-select my_robot_gui
``` 
### Run the node:
First, source the built package. Still in the `ws_urc` folder, run:
``` bash
source install/setup.bash
```
Alternatively, if you are in zsh, run:
``` bash
source install/setup.zsh
``` 
Now you can run the node with:
``` bash
ros2 run my_robot_gui position_control_gui
```
Or, to run the node with custom parameters, run:
``` bash
ros2 run my_robot_gui position_control_gui --ros-args -p '<parameter_name>:=<parameter_value>'
```
Example:
``` bash
ros2 run my_robot_gui position_control_gui --ros-args -p 'range:=["-5.0", "5.0"]' -p 'joint_count:=3'
```
## Executables:
### `position_control_gui`:
The only node in the package. It uses tkinter to create a simple gui containing multiple sliders which can be used to publish rotation vales to the controller topic
#### Parameters:
###### `controller_topic` (string)
The topic which rotation values are published to.
Default value is `/forward_position_controller/commands`.

###### `joint_count` (int32)
The number of joint the node accounts for. The node creates a slider for each joint. Default value is `2`.

###### `range` (string array)
The mininum and maximum values of the sliders. Format as `["<min>", "<max>"]`. Values are converted to Float32. Default values are `["-3.14", "3.14"]`