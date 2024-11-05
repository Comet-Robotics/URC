import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

SIM_MODE = True

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        
        # Subscribing to the depth image topic
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10)
        
        # Subscribing to the regular image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)

    def depth_image_callback(self, msg):
        # Convert depth image from ROS to OpenCV
        try:
            # Convert depth image from ROS to OpenCV
            try:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            except Exception as e:
                self.get_logger().error(f"Could not convert image: {e}")
                return

            min_depth = np.min(depth_image)
            max_depth = np.max(depth_image)
            if max_depth > 1000:
                max_depth = 10
            self.get_logger().info(f'Min depth: {min_depth}, Max depth: {max_depth}')
            
            # Normalize the depth image for display
            depth_image_normalized = (depth_image * 255 / max_depth).astype(np.uint8)
            # Display the depth image
            cv2.imshow('Depth Image', depth_image_normalized)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def image_callback(self, msg):
        # Convert regular image from ROS to OpenCV
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Color Image', color_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process color image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
