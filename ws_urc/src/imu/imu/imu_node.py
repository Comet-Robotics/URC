import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from math import pi as PI
import numpy as np
import serial
# voltage, current, charge

orientation_deg = [0.0,0.0,0.0]
class IMU(Node):
    def __init__(self):
        super().__init__('IMU')
        self.imu_data_publisher = self.create_publisher(
            Imu,
            'imu',
            10
        )
        self.timer = self.create_timer(0.01, self.read_and_send_imu_data);        
        self.orientation_data_publisher = self.create_publisher(
            Vector3,
            'robot_orientation',
            10
        )
        port = self.get_parameter_or('comm_port',"/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_87BD260E51544B5933202020FF0A4126-if00")

        self.get_logger().info("IMU Node started")
        self.get_logger().info(f"Connecting to {port}")
        self.ser = serial.Serial(
            port= port,  # Replace with your port
            baudrate=115200,        # Baud rate
            timeout=1             # Timeout for read operations
        )
        

    def euler_to_quaternion(r, p, y):
            qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)
            qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)
            qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
            qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
            return [qx, qy, qz, qw]

    def read_and_send_imu_data(self):
        
     
        imu_data_msg = Imu()
        orientation_data_msg = Vector3()
        ser_data = self.ser.readline().decode("utf-8").strip().split(':')
        #print(ser_data)
        gyro = ser_data[0].split(',')
        linear_accel = ser_data[1].split(',')
        orientation_quat = ser_data[2].split(',')
        #print(float(gyro[0]))

        imu_data_msg.angular_velocity.x = float(gyro[0])
        imu_data_msg.angular_velocity.y = float(gyro[1])
        imu_data_msg.angular_velocity.z = float(gyro[2])
        imu_data_msg.linear_acceleration.x = float(linear_accel[0])
        imu_data_msg.linear_acceleration.y = float(linear_accel[1])
        imu_data_msg.linear_acceleration.z = float(linear_accel[2])
        imu_data_msg.orientation.x = float(orientation_quat[0])
        imu_data_msg.orientation.y = float(orientation_quat[1])
        imu_data_msg.orientation.z = float(orientation_quat[2])
        imu_data_msg.orientation.w = float(orientation_quat[3])

        orientation_data_msg.x = orientation_deg[0] # roll
        orientation_data_msg.y = orientation_deg[1] # pitch
        orientation_data_msg.z = orientation_deg[2] # yaw

        imu_data_msg._header.frame_id = "child"
        self.imu_data_publisher.publish(imu_data_msg)
        self.orientation_data_publisher.publish(orientation_data_msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = IMU()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
