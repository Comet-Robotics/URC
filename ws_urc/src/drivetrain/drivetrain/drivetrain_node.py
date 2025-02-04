import rclpy
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist

from rclpy.node import Node
import serial
import time


class DrivetrainNode(Node):

    def __init__(self):
        super().__init__('drivetrain_action_server')
        self.declare_parameter("port","/dev/ttyACM0")
        port = self.get_parameter_or("port",Parameter("/dev/serial/by-id/usb-SOLIS_ROVER_PROJECT_DRIVETRAIN-if00"))
        self.get_logger().info(f"Opening on Port {port} ") 
        self.ser = serial.Serial(
            port=port,  # Replace with your port
            baudrate=115200,        # Baud rate
            timeout=1             # Timeout for read operations
        )
        # self.alive_timer = self.create_timer(1.0 , self.send_alive_message)
        self.subscription = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.5, self.heart_beat_callback)
        self.get_logger().info(f"LIVE") 

    def drive_motors(self, linear_x, linear_y, angular_z):
        # Define the robot's geometry
        wheelbase = 0.4  # distance between front and rear wheels, example value
        trackwidth = 0.3  # distance between left and right wheels, example value

        # Compute motor speeds for each Mecanum wheel
        front_left_speed = linear_x - linear_y - (angular_z * (wheelbase + trackwidth) / 2)
        front_right_speed = linear_x + linear_y + (angular_z * (wheelbase + trackwidth) / 2)
        rear_left_speed = linear_x + linear_y - (angular_z * (wheelbase + trackwidth) / 2)
        rear_right_speed = linear_x - linear_y + (angular_z * (wheelbase + trackwidth) / 2)
        self.get_logger().info(f"Right Speed: {front_right_speed} Left Speed: {front_left_speed} Rear Right Speed: {rear_right_speed} Rear Left Speed: {rear_left_speed}")
        # Send speeds to your motor controllers
        self.set_motor_speeds(front_left_speed, front_right_speed, rear_left_speed, rear_right_speed)
    def heart_beat_callback(self):
        self.ser.write(b'apple\n')
    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities from the Twist message
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Drive the motors
        self.drive_motors(linear_x, linear_y, angular_z)
        # Send PWM signals to your motor controllers here


    def set_motor_speeds(
            self,
            front_left_speed,
            front_right_speed,
            rear_left_speed,
            rear_right_speed
            ):
        front_left_speed = int(front_left_speed*10)
        front_right_speed = int(front_right_speed*10)
        rear_left_speed = int(rear_left_speed*10)
        rear_right_speed = int(rear_right_speed*10)

        self.ser.write(f'{rear_right_speed}:{rear_left_speed}:{front_right_speed}:{front_left_speed}\n'.encode())



def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = DrivetrainNode()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()