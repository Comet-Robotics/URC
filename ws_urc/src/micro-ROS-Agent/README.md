# micro-ROS Agent

ROS 2 package using Micro XRCE-DDS Agent. Credit to this REPO https://github.com/micro-ROS/micro-ROS-Agent

## Overview

This repository contains the Micro-ROS Agent package.

To attach a microcontroller that is already flashed with micro-ros run

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Replace the port with the port the microcontroller is attached to.

This agent acts as a middle man between the microcontrollers and the main computer. **You will likely never have to touch this package**

Everything below this line is from the original package

## Package features

### XML generation

During the build process, the package looks for all ROS 2 messages to generate an initial list of XML profiles.
These profiles can are referenced in the Agent-Client communication to avoid sending the full XML content.
This reference mechanism can be switched on and off from the Micro XRCE-DDS middleware layer.

### Agent-Client communication mechanism

Communication between the Micro-ROS Agent and the Micro-ROS nodes supports two types of transport:

- UDP and TCP over IPv4 and IPv6.
- Serial Port transports.

All available configurations are supported directly by the Micro XRCE-DDS agent.

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in this repository,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

Please notice the following issues/limitations:

* There is an unknown issue when dealing with serial ports shared with the micro-ROS agent running inside a Docker. Sometimes it works with a remarkable packet loss.
* There is an known issues with serial port communication on micro-ros-agent snap version. It is recommended to use the dockerized version or build it from source.