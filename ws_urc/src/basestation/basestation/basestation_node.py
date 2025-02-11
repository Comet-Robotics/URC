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
# Generate the protobuf code
import sys
from . import msgs_pb2  # Assuming your proto file is named message.proto

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
        self.imu_subscription  # prevent unused variable warning
        self.gps_subscription  # prevent unused variable warning

        self.server_address = (server_ip, 8000)  # Use IP from argument
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.get_logger().info(f"Connecting to {self.server_address}")
 
            self.sock.connect(self.server_address)
            self.get_logger().info(f"Connected to {self.server_address}")
        except socket.error as e:
            self.get_logger().error(f"Connection failed: {e}")
            self.sock = None # Disable sending if connection fails

    def imu_callback(self, msg):
        if self.sock is None:
            return
        
    

        imu_data = msgs_pb2.IMUData()

        # Header
        imu_data.header.seq = msg.header.stamp.nanosec
        imu_data.header.frame_id = msg.header.frame_id
        imu_data.header.stamp = self.ros_time_to_iso_string(msg.header.stamp)

        # Orientation
        imu_data.orientation.x = msg.orientation.x
        imu_data.orientation.y = msg.orientation.y
        imu_data.orientation.z = msg.orientation.z
        imu_data.orientation.w = msg.orientation.w

        # Orientation Covariance
        # imu_data.orientation_covariance.extend(msg.orientation_covariance)

        # Angular Velocity
        # imu_data.angular_velocity.x = msg.angular_velocity.x
        # imu_data.angular_velocity.y = msg.angular_velocity.y
        # imu_data.angular_velocity.z = msg.angular_velocity.z

        # # Angular Velocity Covariance
        # imu_data.angular_velocity_covariance.extend(msg.angular_velocity_covariance)

        # # Linear Acceleration
        # imu_data.linear_acceleration.x = msg.linear_acceleration.x
        # imu_data.linear_acceleration.y = msg.linear_acceleration.y
        # imu_data.linear_acceleration.z = msg.linear_acceleration.z

        # # Linear Acceleration Covariance
        # imu_data.linear_acceleration_covariance.extend(msg.linear_acceleration_covariance)

        # Create a Message and set the IMU data
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
        gps_data.ground_speed = msg.ground_speed  # Assuming track is ground_speed
        gps_data.satellites = msg.satellites
        gps_data.mode_indicator = msg.mode_indicator# Assuming this is the correct mapping
        gps_data.separation = msg.separation
        gps_data.true_course = msg.true_course # Using track for true course
        gps_data.true_course_magnetic = msg.true_course_magnetic # Using track for magnetic course
        gps_data.dilution = msg.dilution # Assuming this is the correct mapping
        gps_data.utc_time = msg.utc_time # Not directly available, using system time

        # Create a Message and set the GPS data
        message = msgs_pb2.Message()
        message.gps.CopyFrom(gps_data)

        self.send_protobuf_message(message)

    def send_protobuf_message(self, message):
        """Serializes and sends a protobuf message over TCP."""
        try:
            serialized_data = message.SerializeToString()
            # Prepend the message with its length
            message_length = len(serialized_data)
            packed_length = struct.pack('!I', message_length)  # Network byte order (Big-Endian)
            self.sock.sendall(packed_length + serialized_data)
            self.get_logger().debug(f"Sent message of length: {message_length}")
        except socket.error as e:
            self.get_logger().error(f"Error sending data: {e}")
            self.sock.close()
            self.sock = None # Disable sending

    def ros_time_to_iso_string(self, time_msg):
        """Converts a ROS Time message to an ISO 8601 string."""
        seconds = time_msg.sec
        nanoseconds = time_msg.nanosec
        total_seconds = seconds + nanoseconds / 1e9
        return datetime.datetime.fromtimestamp(total_seconds).isoformat()

    def destroy_node(self):
        """Called when the node is being shut down."""
        if self.sock:
            self.sock.close()
            self.get_logger().info("Socket closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='ROS 2 Data Forwarder Node')
    parser.add_argument('ip', type=str, help='IP address of the server')
    args = parser.parse_args()

    data_forwarder = DataForwarder(args.ip)  # Pass the IP to the constructor

    rclpy.spin(data_forwarder)
    data_forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()