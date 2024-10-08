import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import serial
from action_drivetrain_interface.action import Forward # type: ignore
from action_drivetrain_interface.action import Turn # type: ignore

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('drivetrain_action_server')
        self.forward_action_server = ActionServer(
            self,
            Forward,
            'forward',
            self.forward_execute_callback)
        self.turn_action_server = ActionServer(
            self,
            Turn,
            'turn',
            self.turn_execute_callback)
        self.ser = serial.Serial(
            port='/dev/ttyACM0',  # Replace with your port
            baudrate=115200,        # Baud rate
            timeout=1             # Timeout for read operations
        )
        self.alive_timer = self.create_timer(1.0 , self.send_alive_message)


    def send_alive_message(self):
        self.ser.write(b'ALIVE\n')
        self.get_logger().info('Sent Alive Message via Serial')

    def forward_execute_callback(self, goal_handle):
        self.get_logger().info('Starting path...')
        feedback_msg = Forward.Feedback()
        feedback_msg.traveled = 0

        # The loop below is just a prototype to apply gradual speed ramping for the wheels, to try and prevent jerky movements
        for i in range(0, 25, 5):
            self.ser.write(f'{i}:{i}\n'.encode())
            rclpy.sleep(1)

        self.ser.write(b'20:20\n')

        for i in range(1, goal_handle.request.distance):
            feedback_msg.traveled = i
            
            self.get_logger().info('Feedback: {0}'.format(i))
            goal_handle.publish_feedback(feedback_msg)
            rclpy.sleep(1)
        self.ser.write(b'0:0\n')
        goal_handle.succeed()

        result = Forward.Result()
        result.total_distance = feedback_msg.traveled
        return result
    


    def turn_execute_callback(self, goal_handle):
        self.get_logger().info('Starting left turn...')
        feedback_msg = Turn.Feedback()
        feedback_msg.degrees_moved = 0
        if (goal_handle.request.degree > 0):
            self.ser.write(b'-20:20\n')
        else:
            self.ser.write(b'20:-20\n')

        try:

            for i in range(1, abs(goal_handle.request.degree)):
                feedback_msg.degrees_moved = i
                
                self.get_logger().info('Feedback: {0}'.format(i))
                goal_handle.publish_feedback(feedback_msg)
                rclpy.sleep(0.025)
            self.ser.write(b'0:0\n')
            goal_handle.succeed()

            result = Turn.Result()
            result.total_degrees = feedback_msg.degrees_moved
            return result
        except:
            self.ser.write(b'0:0\n')
    


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()