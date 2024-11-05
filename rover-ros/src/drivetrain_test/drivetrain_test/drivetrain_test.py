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
        self.turn_action_client=ActionClient(
            self, 
            Turn, 
            'turn')
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
            feedback_callback=self.feedback_callback
            )

    def send_turn(self,turn):
        self.get_logger().info('Sending turn: %d' % turn) 
        goal_msg = Turn.Goal()

        self.turn_action_client.wait_for_server()
        self.turn_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback2,
            # result_callback=self.result_callback2
            )


    def feedback_callback(self, feedback):
        self.stdscr.addstr('Feedback: %d' % feedback.feedback.traveled)

    
    def feedback_callback2(self, feedback):
        self.stdscr.addstr('Feedback: %d' % feedback.feedback.degrees_moved)

    # def result_callback(self, future):
    #     result = future.result().result
    #     self.stdscr.addstr('Result: %d' % result.total_distance)

    
    def result_callback2(self, future):
        result = future.result().result
        self.stdscr.addstr('Result: %d' % result.total_degrees)

    def detect_keypresses(self):
        a = self.stdscr.getch()
        self.stdscr.clear()
        if a != -1:
            self.stdscr.addstr(f'You pressed: {chr(a)}')
            self.stdscr.refresh()
            if (chr(a) == 'w'):
                self.send_goal(2)
            elif (chr(a) == 's'):
                self.send_goal(-2)
            elif (chr(a) == 'a'):
                self.send_turn(2)
            elif (chr(a) == 'd'):
                self.send_turn(-2)
            else:
                self.send_turn(0)
                self.send_goal(0)
        

     
def start(stdscr):
    rclpy.init()
    drivetrain_action_client_test = DrivetrainActionClient(stdscr)
    rclpy.spin(drivetrain_action_client_test)


def main():
    curses.wrapper(start)

if __name__ == '__main__':
    main()