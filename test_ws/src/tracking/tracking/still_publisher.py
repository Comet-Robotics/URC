import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os

from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('tracking')

class ImagePublisher(Node):
    def __init__(self, Hz=1):
        super().__init__("image_publisher")
        self.publisher = self.create_publisher(Image, "camera/image_raw", 10)
        self.create_timer(1/Hz, self.timer_callback)

        self.bridge = CvBridge()
        self.get_logger().info('Camera publisher started.')

    def timer_callback(self):
        img_path = os.path.join(pkg_share, 'img', 'Human_faces.jpg')
        img = cv2.imread(img_path)        

        # Convert form OpenCV image (numpy array) to Image Message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher.publish(img_msg)
        self.get_logger().info('Published Frame.')
    
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
