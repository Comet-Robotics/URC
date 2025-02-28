# radiodriver-proxy

Service which will be used by the base station and ROS nodes to handle logic for sending messages to the [radio driver](http://github.com/comet-robotics/urc-radiodriver). basestation and ROS nodes send messages to this process via ____ (TBD... websockets? tcp? unix pipes? unix sockets?), then this process handles message queueing and sending to the radio driver via serial.

There is additional logic needed to wrap messages before they are sent to the radio driver which this process handles, reducing redundant code in each component.
