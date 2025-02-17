import rclpy
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
        self.connect_retry_timer = self.create_timer(5.0, self.try_reconnect)
        self.connect_retry_timer.cancel()

        self.cmd_vel_publisher = self.create_publisher(RosTwist, '/cmd_vel_manual', 10)
        self.receive_thread = threading.Thread(target=self.receive_protobuf_messages)
        self.receive_thread.start()

        self.connect_to_server()

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
            self.get_logger().warn("Received NaN value in GPS data, skipping message.")
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

    def send_protobuf_message(self, message):
        try:
            serialized_data = message.SerializeToString()
            message_length = len(serialized_data)
            packed_length = struct.pack('!I', message_length)
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
        if self.sock:
            self.sock.close()
            self.get_logger().info("Socket closed.")
        self.turn_off_motors()
        self.connect_retry_timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    data_forwarder = DataForwarder()

    rclpy.spin(data_forwarder)
    data_forwarder.destroy_node()
    data_forwarder.receive_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()