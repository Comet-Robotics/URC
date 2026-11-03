import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SplitterNode(Node):
    def __init__(self):
        super().__init__('splitter_node')

        # Subscribers
        self.pwm_sub = self.create_subscription(
            Float32MultiArray,
            '/pwm',
            self.pwm_callback,
            10
        )

        self.rpm_sub = self.create_subscription(
            Float32MultiArray,
            '/rpm',
            self.rpm_callback,
            10
        )

        # Publishers
        self.pwm_left_pub = self.create_publisher(Float32MultiArray, '/pwm_left', 10)
        self.pwm_right_pub = self.create_publisher(Float32MultiArray, '/pwm_right', 10)

        self.rpm_left_pub = self.create_publisher(Float32MultiArray, '/rpm_left', 10)
        self.rpm_right_pub = self.create_publisher(Float32MultiArray, '/rpm_right', 10)

    def split_even_odd(self, data):
        # Even indices -> left, Odd indices -> right
        left = [data[i] for i in range(0, len(data), 2)]
        right = [data[i] for i in range(1, len(data), 2)]
        return left, right

    def pwm_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn('PWM array too small')
            return

        left, right = self.split_even_odd(msg.data)

        left_msg = Float32MultiArray()
        right_msg = Float32MultiArray()

        left_msg.data = left
        right_msg.data = right

        self.pwm_left_pub.publish(left_msg)
        self.pwm_right_pub.publish(right_msg)

    def rpm_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn('RPM array too small')
            return

        left, right = self.split_even_odd(msg.data)

        left_msg = Float32MultiArray()
        right_msg = Float32MultiArray()

        left_msg.data = left
        right_msg.data = right

        self.rpm_left_pub.publish(left_msg)
        self.rpm_right_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SplitterNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()