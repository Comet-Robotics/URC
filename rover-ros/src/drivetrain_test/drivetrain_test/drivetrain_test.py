import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import serial
from action_drivetrain_interface.action import Forward # type: ignore
from action_drivetrain_interface.action import Turn # type: ignore
import curses

class DrivetrainActionClient(Node):

    def __init__(self,stdscr):
        super().__init__('drivetrain_test')
        self.action_client=ActionClient(
            self, 
            Forward, 
            'forward')
        stdscr.nodelay(True)
        self.stdscr=stdscr
        self.keypress_timer=self.create_timer(
            0.1,
            self.detect_keypresses)

    def send_goal(self, distance):
        self.get_logger().info('Sending goal: %d' % distance) 
        goal_msg = Forward.Goal()
        goal_msg.distance = distance

        self.action_client.wait_for_server()
        self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Feedback: %d' % feedback.feedback.progress)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: %d' % result.sum)

    def detect_keypresses(self):
        a = self.stdscr.getch()
        self.stdscr.clear()
        if a != -1:
            self.stdscr.addstr(f'You pressed: {chr(a)}')
            self.stdscr.refresh()

     
def start(stdscr):
    rclpy.init()
    drivetrain_action_client_test = DrivetrainActionClient(stdscr)
    rclpy.spin(drivetrain_action_client_test)


def main():
    curses.wrapper(start)

if __name__ == '__main__':
    main()