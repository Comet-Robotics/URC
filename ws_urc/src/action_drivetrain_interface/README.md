# `action_drivetrain_interface`

This ROS2 package is used to define the action message types used to communication the state, result, and callback values for actions relating the the operation of the drivebase.

## Forward Action

This is the action message used to define the messages sent when moving the drivebase forwards:

```action
int32 distance
---
int32 total_distance
---
int32 traveled
```

Breaking this down:

`int32 distance` - The amount of distance to travel in the forward direction

`int32 total_distance` - The total amount of distance traversed by the rover for the duration of this action

`int 32 traveled` - The current amount of distance traversed by the rover

## Turn Action

```action
int32 degree
---
int32 total_degrees
---
int32 degrees_moved
```

Breaking this down:

`int32 degree` - The heading to turn the robot to; objectivity (global, robot-relative, etc.) depends on action implementation

`int32 total_degrees` - The total amount of degrees traversed by the rover for the duration of this action

`int32 degrees_moved` - The current amount of degrees traversed by the rover

## Usage & Summary

Use these interfaces to direct the robot drivetrain by implementing actions that properly make use of the action message types.

Add this package to your ROS2 package using the following:

```xml
<exec_depend>action_drivetrain_interface<exec_depend>
```
