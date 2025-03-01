# radiodriver-proxy

This service is for the the base station and ROS nodes to handle logic for sending and receiving messages with the [radio driver](http://github.com/comet-robotics/urc-radiodriver), as an alternative/backup option for the Wi-Fi TCP uplink. There is additional logic needed to [un-]wrap messages before they are sent/received to the radio driver which this process handles, removing the need to implement serial support and KISS framing support in both the base station and ROS nodes. Both the base station and rover will have local instances of this service running.

To send a message via radio, other processes simply write an integer representing the size of the message in bytes, followed by the message data, to the Unix socket at `/var/run/radiodriver-proxy.sock`. The proxy will receive the message, KISS frame it, and send it to the radio driver via serial for transmission.

When the radio driver receives a message from the radio, it sends the message back to this proxy over serial. Then, the proxy will unframe the message, and send it to the receiving process via Unix socket. The receiving process will listen for data on the Unix socket at `/var/run/radiodriver-proxy.sock`, again consisting of an integer, representing the size of the message in bytes, followed by the message data.

