import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import serial
from action_drivetrain_interface.action import Drivetrain 
import time

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('drivetrain_action_server')
        self._action_server = ActionServer(
            self,
            Drivetrain,
            'forward',
            self.execute_callback)
        self.ser = serial.Serial(
            port='/dev/ttyACM0',  # Replace with your port
            baudrate=115200,        # Baud rate
            timeout=1             # Timeout for read operations
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Starting path...')
        feedback_msg = Drivetrain.Feedback()
        feedback_msg.traveled = 0
        self.ser.write(b'20:20\n')

        for i in range(1, goal_handle.request.distance):
            feedback_msg.traveled = i
            
            self.get_logger().info('Feedback: {0}'.format(i))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        self.ser.write(b'0:0\n')
        goal_handle.succeed()

        result = Drivetrain.Result()
        result.total_distance = feedback_msg.traveled
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()