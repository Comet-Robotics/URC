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
import datetime
import argparse
import math
import time
from . import msgs_pb2
class DataForwarder(Node):

    def __init__(self, server_ip):
        super().__init__('data_forwarder')
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
        self.connect_to_server()

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
        self.get_logger().info(f"GPS Data - Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}")

        gps_data.longitude = None if math.isnan(msg.longitude) else float(msg.longitude)
        gps_data.latitude =  None if math.isnan(msg.latitude) else float(msg.latitude)
        gps_data.altitude =  None if math.isnan(msg.altitude) else float(msg.altitude)
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

    def ros_time_to_iso_string(self, time_msg):
        seconds = time_msg.sec
        nanoseconds = time_msg.nanosec
        total_seconds = seconds + nanoseconds / 1e9
        return datetime.datetime.fromtimestamp(total_seconds).isoformat()

    def destroy_node(self):
        if self.sock:
            self.sock.close()
            self.get_logger().info("Socket closed.")
        self.connect_retry_timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='ROS 2 Data Forwarder Node')
    parser.add_argument('ip', type=str, help='IP address of the server')
    args = parser.parse_args()

    data_forwarder = DataForwarder(args.ip)

    rclpy.spin(data_forwarder)
    data_forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()