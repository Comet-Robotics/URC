import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Imu
from gpsx.msg import Gpsx
import socket
import struct
from time import time
from geometry_msgs.msg import Vector3 as RosVector3
from geometry_msgs.msg import Quaternion as RosQuaternion
from std_msgs.msg import Header as RosHeader
from geometry_msgs.msg import Twist as RosTwist
import datetime
import argparse
import math
import time
from . import msgs_pb2
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import threading
import base64

class DataForwarder(Node):

    def __init__(self):
        super().__init__('data_forwarder')
 
        ip_descriptor = ParameterDescriptor(description='IP address of the server')
        self.declare_parameter('ip', '127.0.0.1', ip_descriptor)
        server_ip = self.get_parameter('ip').get_parameter_value().string_value
        self.sock = None
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.gps_subscription = self.create_subscription(
            Gpsx,
            '/gpsx',
            self.gps_callback,
            10)
        self.imu_subscription
        self.gps_subscription

        self.server_address = (server_ip, 8000)
        self.server_ip = server_ip
        self.connect_retry_timer = self.create_timer(1.0, self.try_reconnect)
        self.connect_retry_timer.cancel()

        self.cmd_vel_publisher = self.create_publisher(RosTwist, '/cmd_vel_manual', 10)
        self.serial = open("/dev/ttyACM0", "rw")
        self.receive_thread = threading.Thread(target=self.receive_protobuf_messages)
        self.receive_thread.start()
        self.connect_to_server()


    def cleanup(self):
        """Perform cleanup operations."""
        self.turn_off_motors()
        
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception as e:
                self.get_logger().error(f"Error closing socket: {e}")
        
        if hasattr(self, 'connect_retry_timer') and not self.connect_retry_timer.is_canceled():
            self.connect_retry_timer.cancel()
        
        self.get_logger().info("Cleanup completed")

    def connect_to_server(self):
        if self.sock:
            self.sock.close()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        

        try:
            self.get_logger().info(f"Connecting to {self.server_address}")
            self.sock.connect(self.server_address)
            self.get_logger().info(f"Connected to {self.server_address}")
            self.connect_retry_timer.cancel()
        except socket.error as e:
            self.get_logger().error(f"Connection failed: {e}")
            self.sock = None
            self.connect_retry_timer.reset()

    def try_reconnect(self):
        self.get_logger().info("Attempting to reconnect...")
        self.turn_off_motors()
        self.connect_to_server()

    def turn_off_motors(self):
        twist = RosTwist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Motors turned off, published zero Twist message to /cmd_vel_manual")

    def imu_callback(self, msg):
        if self.sock is None:
            return

        imu_data = msgs_pb2.IMUData()

        imu_data.header.seq = msg.header.stamp.nanosec
        imu_data.header.frame_id = msg.header.frame_id
        imu_data.header.stamp = self.ros_time_to_iso_string(msg.header.stamp)

        imu_data.orientation.x = msg.orientation.x
        imu_data.orientation.y = msg.orientation.y
        imu_data.orientation.z = msg.orientation.z
        imu_data.orientation.w = msg.orientation.w

        message = msgs_pb2.Message()
        message.imu.CopyFrom(imu_data)

        self.send_protobuf_message(message)

    def gps_callback(self, msg):
        if self.sock is None:
            return

        gps_data = msgs_pb2.GPSData()
        

        if math.isnan(msg.longitude) or math.isnan(msg.latitude) or math.isnan(msg.altitude):
            return

        gps_data.longitude = float(msg.longitude)
        gps_data.latitude = float(msg.latitude)
        gps_data.altitude = float(msg.altitude)
        gps_data.ground_speed = msg.ground_speed
        gps_data.satellites = msg.satellites
        gps_data.mode_indicator = msg.mode_indicator
        gps_data.separation = msg.separation
        gps_data.true_course = msg.true_course
        gps_data.true_course_magnetic = msg.true_course_magnetic
        gps_data.dilution = msg.dilution
        gps_data.utc_time = msg.utc_time

        message = msgs_pb2.Message()
        message.gps.CopyFrom(gps_data)

        self.send_protobuf_message(message)


    #add some stuff to send and receive to do ham instead
    def send_protobuf_message(self, message):
        serialized_data = message.SerializeToString()
        message_length = len(serialized_data)
        packed_length = struct.pack('!I', message_length)

        flag = True
        if flag:
            try:
                self.serial.write(base64.b64encode(packed_length + serialized_data) + "\n")
                self.get_logger().debug(f"Sent message of length: {message_length}")
            except socket.error as e:
                self.get_logger().error(f"Error sending data: {e}")
        else:
            try:
                self.sock.sendall(packed_length + serialized_data)
                self.get_logger().debug(f"Sent message of length: {message_length}")
            except socket.error as e:
                self.get_logger().error(f"Error sending data: {e}")
                if self.sock:
                    self.sock.close()
                self.sock = None
                self.connect_retry_timer.reset()

    
    def receive_protobuf_messages(self):
        while rclpy.ok():
            flag = True
            if flag:
                line = self.serial.readline()
                line : str
                if "[PACKET RX]" in line:
                    msg = line[line.find("[PACKET RX]")+11:]
                    msg = base64.b64decode(msg)
                    packed_length = msg[:4]
                    message_length = struct.unpack('!I', packed_length)[0]
                    serialized_data = msg[4:message_length+4]
                    message = msgs_pb2.Message()
                    message.ParseFromString(serialized_data)
                    self.handle_received_message(msg)


            else:
                if self.sock is None:
                    time.sleep(1)
                    continue

                try:
                    packed_length = self.sock.recv(4)
                    if not packed_length:
                        continue
                    message_length = struct.unpack('!I', packed_length)[0]
                    serialized_data = self.sock.recv(message_length)
                    message = msgs_pb2.Message()
                    message.ParseFromString(serialized_data)
                    self.handle_received_message(message)
                except socket.error as e:
                    self.get_logger().error(f"Error receiving data: {e}")
                    self.sock = None
                    self.connect_retry_timer.reset()

    def handle_received_message(self, message):
        if message.HasField('twist'):
            self.publish_twist_message(message.twist)
        # if message is radio_change

    def publish_twist_message(self, twist_msg):
        twist = RosTwist()
        twist.linear.x = twist_msg.linear.x
        twist.linear.y = twist_msg.linear.y
        twist.linear.z = twist_msg.linear.z
        twist.angular.x = twist_msg.angular.x
        twist.angular.y = twist_msg.angular.y
        twist.angular.z = twist_msg.angular.z
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Published Twist message to /cmd_vel")

    def ros_time_to_iso_string(self, time_msg):
        seconds = time_msg.sec
        nanoseconds = time_msg.nanosec
        total_seconds = seconds + nanoseconds / 1e9
        return datetime.datetime.fromtimestamp(total_seconds).isoformat()

    def destroy_node(self):
        """Enhanced destroy_node with proper cleanup."""
        # self.cleanup()
        if hasattr(self, 'receive_thread') and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
            if self.receive_thread.is_alive():
                self.get_logger().warning("Receive thread did not terminate gracefully")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        data_forwarder = DataForwarder()
        rclpy.spin(data_forwarder)
    except Exception as e:
        if data_forwarder:
            data_forwarder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()