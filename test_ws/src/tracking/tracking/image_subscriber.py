import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_display")
        self.subscriber = self.create_subscription(
            Image, 'camera/image_raw', self.show_image, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Subscribing to raw image")

    def show_image(self, msg):
        self.get_logger().info("Recieved Frame")
        image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("Video", image)
        cv2.waitKey(1)
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()