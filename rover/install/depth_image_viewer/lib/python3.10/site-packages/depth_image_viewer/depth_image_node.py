import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthImageViewer(Node):
    def __init__(self):
        super().__init__('depth_image_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Display the depth image
        cv2.imshow("Depth Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DepthImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
