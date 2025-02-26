import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import rclpy.time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
import serial
from typing import List, Optional
orientation_deg = [0.0,0.0,0.0]
class IMU(Node):
    """IMU node for reading and publishing IMU data."""

    def __init__(self):
        super().__init__('IMU')
        
        self.declare_parameter('comm_port', '/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_87BD260E51544B5933202020FF0A4126-if00')
        self.declare_parameter('baud_rate', 115200)
        
        self.imu_data_publisher = self.create_publisher(Imu, 'imu', 10)
        self.orientation_data_publisher = self.create_publisher(Vector3, 'robot_orientation', 10)
        
        try:
            self.setup_serial_connection()
            self.timer = self.create_timer(0.01, self.read_and_send_imu_data)
            self.get_logger().info("IMU Node started successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize serial connection: {e}")
            raise

        self.error_count = 0
        self.MAX_CONSECUTIVE_ERRORS = 5  # Kill connection after this many errors

    def setup_serial_connection(self) -> None:
        """Initialize serial connection with configured parameters."""
        port = self.get_parameter('comm_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        self.get_logger().info(f"Connecting to {port} at {baud_rate} baud")
        self.ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=1
        )

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> List[float]:
        """Convert euler angles to quaternion."""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def shutdown_connection(self) -> None:
        """Safely shutdown the connection and stop the timer."""
        
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.get_logger().warning("IMU connection has been terminated due to persistent errors")

    def read_and_send_imu_data(self) -> None:
        """Read data from serial port and publish to ROS topics."""
        imu_data_msg = Imu()
        orientation_data_msg = Vector3()
        
        try:
            ser_data = self.ser.readline().decode("utf-8").strip()
            gyro, linear_accel, orientation_quat = [part.split(',') for part in ser_data.split(':')]

            # Set angular velocity
            imu_data_msg.angular_velocity.x = float(gyro[0])
            imu_data_msg.angular_velocity.y = float(gyro[1])
            imu_data_msg.angular_velocity.z = float(gyro[2])

            # Set linear acceleration
            imu_data_msg.linear_acceleration.x = float(linear_accel[0])
            imu_data_msg.linear_acceleration.y = float(linear_accel[1])
            imu_data_msg.linear_acceleration.z = float(linear_accel[2])

            # Set orientation
            imu_data_msg.orientation.x = float(orientation_quat[0])
            imu_data_msg.orientation.y = float(orientation_quat[1])
            imu_data_msg.orientation.z = float(orientation_quat[2])
            imu_data_msg.orientation.w = float(orientation_quat[3])

            # Set orientation degrees
            orientation_data_msg.x = float(orientation_deg[0])  # roll
            orientation_data_msg.y = float(orientation_deg[1])  # pitch
            orientation_data_msg.z = float(orientation_deg[2])  # yaw

            # Set header
            imu_data_msg.header.frame_id = "imu_frame"
            imu_data_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish messages
            self.imu_data_publisher.publish(imu_data_msg)
            self.orientation_data_publisher.publish(orientation_data_msg)

            # Reset error count on successful read
            self.error_count = 0

        except (ValueError, IndexError, serial.SerialException) as e:
            self.error_count += 1
            self.get_logger().error(f"Error reading IMU data: {e}")
            
            if self.error_count >= self.MAX_CONSECUTIVE_ERRORS:
                self.get_logger().error("Too many consecutive errors, shutting down IMU connection")
                raise RuntimeError("IMU connection terminated due to persistent errors")

    def __del__(self) -> None:
        """Cleanup when the node is destroyed."""
        self.shutdown_connection()

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    node = IMU()
    try:
        rclpy.spin(node)
    except Exception as e:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
