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
from threading import Lock  
import base64
import serial  # Add this import


class Connection:
    def __init__(self, ip, port, serial_port, baud_rate,logger):
        self.ip = ip
        self.port = port
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.sock = None
        self.serial = None
        self.logger = logger
    def get_connected(self):
        return self.sock is not None or self.serial is not None
    def connect_tcp(self):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self.ip, self.port))
            self.sock = sock
    def connect_serial(self):
            self.serial = serial.Serial(self.serial_port, self.baud_rate)
            self.logger.info("Connected to serial port")

    def send_serial(self, message):
        serialized_data = message.SerializeToString()
        encoded_data = base64.b64encode(serialized_data) + b'\n'
        self.logger.info(f"Encoded Data {encoded_data}")

        wrote = self.serial.write(encoded_data)
        return wrote
    
    def send_tcp(self, message):    
        
        serialized_data = message.SerializeToString()
        message_length = len(serialized_data)
        packed_length = struct.pack('!I', message_length)
        self.sock.sendall(packed_length + serialized_data)
    
    def recv_tcp(self):
        
        packed_length = self.sock.recv(4)
        message_length = struct.unpack('!I', packed_length)[0]
        serialized_data = self.sock.recv(message_length)
        message = msgs_pb2.Message()
        message.ParseFromString(serialized_data)
        return message

    def recv_serial(self):
        line = str(self.serial.readline())
        if "[PACKET RX]" in line:
            msg = line[line.find("[PACKET RX]")+11:]
            msg = base64.b64decode(msg)
            message = msgs_pb2.Message()
            message.ParseFromString(msg)
            return message
        pass
    
    def send(self, message):
        if self.sock is not None:
                try: 
                    self.send_tcp(message)
                except socket.error as e:
                    self.sock = None

        elif self.serial is not None:
                try:
                    self.send_serial(message)
                except serial.SerialException as e:
                    self.serial = None
        else:
            self.logger.info("No connection to server")




    def receive(self):
        message = None
        if self.sock is not None :
      
            try:
                message = self.recv_tcp()
            except socket.error as e:
                self.logger.error(f'Tcp connection lost: {e}')
                self.sock = None
        elif self.serial is not None:
            
            try:
                message = self.recv_serial()
            except serial.SerialException as e:
                self.logger.error(f'Serial connection lost: {e}')
                self.serial = None
       
      
        return message
            
        
        

    def close(self):
        self.sock.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class DataForwarder(Node):

    def __init__(self):
        super().__init__('data_forwarder')
 
        # Parameter declarations
        ip_descriptor = ParameterDescriptor(description='IP address of the server')
        serial_port_descriptor = ParameterDescriptor(description='Serial port device path')
        baud_rate_descriptor = ParameterDescriptor(description='Serial port baud rate')
        
        self.declare_parameter('ip', '127.0.0.1', ip_descriptor)
        self.declare_parameter('serial_port', '/dev/serial/by-id/', serial_port_descriptor)
        self.declare_parameter('baud_rate', 115200, baud_rate_descriptor)
        
        server_ip = self.get_parameter('ip').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.conn = Connection(server_ip, 8000, serial_port, baud_rate, self.get_logger())
#        self.conn.connect_serial()
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            100)
        self.gps_subscription = self.create_subscription(
            Gpsx,
            '/gpsx',
            self.gps_callback,
            500)
        self.imu_subscription
        self.gps_subscription

        self.server_address = (server_ip, 8000)
        self.server_ip = server_ip
        self.connect_retry_timer = self.create_timer(1.0, self.try_reconnect)

        self.cmd_vel_publisher = self.create_publisher(RosTwist, '/cmd_vel_manual', 10)
            
        self.receive_thread = threading.Thread(target=self.recieve_protobuf_messages)
        self.receive_thread.start()


    def try_reconnect(self):
        if self.conn.sock is None:
            
            self.turn_off_motors()
            self.get_logger().info("Attempting to reconnect...")
            try:
                self.conn.connect_tcp()
            except socket.error as e:
                self.get_logger().error(f'Failed to connect to server: {e}')
                return
        

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
        if self.conn.get_connected() == False:
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
        self.get_logger().info("Recieved IMU message")
        self.conn.send(message)

    def gps_callback(self, msg):
        if self.conn.get_connected() == False:
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
        self.get_logger().info("Recieved GPS message")
        self.conn.send(message)





    def recieve_protobuf_messages(self):
        while True:
            message = self.conn.receive()
            
            if message is not None:
                self.get_logger().info("Received message")
                self.handle_received_message(message)
            
        pass


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
    
    data_forwarder = DataForwarder()
    rclpy.spin(data_forwarder)

    if data_forwarder:
        data_forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
