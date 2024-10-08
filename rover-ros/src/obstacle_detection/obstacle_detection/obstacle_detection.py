import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import numpy as np


class ObstacleDetection(Node):

    def __init__(self):
        super().__init__('obstacle_detection')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("depth",cv_image)
        cv2.waitKey(2)

    def preprocess(image):
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) #rgb -> hsv conversion

        final_img = np.zeros_like(image) #initialize final image

        #initialize kernels
        k_size = 5
        opening_kernel = cv2.getStructuringElement(cv2.MORPH_OPEN, (k_size, k_size))
        closing_kernel = cv2.getStructuringElement(cv2.MORPH_CLOSE, (k_size, k_size))

        for channel in range(3):
            blurred = cv2.medianBlur(hsv_img[:, :, channel], k_size)

            closed = cv2.morphologyEx(blurred, cv2.MORPH_CLOSE, closing_kernel)

            opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, opening_kernel)

            thresh = cv2.adaptiveThreshold(opened, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, blockSize=9, C=4)

            final_img[thresh == 255] = 255 


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()