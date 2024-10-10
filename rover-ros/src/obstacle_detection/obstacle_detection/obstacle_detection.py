import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import numpy as np


class ObstacleDetection(Node):
    """Parameters"""
    #depth_map
    depth_image = None

    #segmentation
    k_size = 5 
    block_size = 9
    gauss_threshold = 2

    #feature extraction
    extract_threshold = 1
    non_max_suppression = True
    obstacle_threshold = 0

    class Obstacle():
        def __init__(self,x,y,height):
            self.height = height
            self.depth = 0
            self.confidence = 0

        def get_depth(self,x,y):
            return 


    def __init__(self):
        super().__init__('obstacle_detection')

        # Subscribe to color and depth topics
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        self.bridge = CvBridge()

    def color_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # cv2.imshow("color",cv_image)
        # segmented, boxes = self.segmentation(cv_image)
        # extracted = self.feature_extraction(cv_image, boxes)
        # cv2.imshow("extracted features", segmented)
        # cv2.waitKey(2)

    def depth_callback(self, msg):
        # Convert depth image to CV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        #cv2.imshow("depth image", depth_image)

    def segmentation(self, image):
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) #rgb -> hsv conversion

        final_img = np.zeros_like(image) #initialize final image

        #initialize kernels
        opening_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size, self.k_size))
        closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size, self.k_size))
        dilation_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size, self.k_size))

        for channel in range(3):
            blurred = cv2.medianBlur(hsv_img[:, :, channel], self.k_size)

            closed = cv2.morphologyEx(blurred, cv2.MORPH_CLOSE, closing_kernel)

            opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, opening_kernel)

            thresh = cv2.adaptiveThreshold(opened, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, blockSize=self.block_size, C=self.gauss_threshold)

            final_img[thresh == 255] = 255 
        
        final_opened = cv2.morphologyEx(final_img, cv2.MORPH_OPEN, opening_kernel)
        final_dilated = cv2.morphologyEx(final_opened, cv2.MORPH_DILATE, dilation_kernel)
        final_grayscale = cv2.cvtColor(final_dilated, cv2.COLOR_BGR2GRAY)

        contours, _ = cv2.findContours(final_grayscale, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []

        # Draw bounding boxes around each contour
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2) 
            boxes.append((x,y,w,h))
        
        return image, boxes


    def feature_extraction(self, image, boxes):
        fast = cv2.FastFeatureDetector.create(self.extract_threshold, self.non_max_suppression)

        for (x, y, w, h) in boxes:
            roi = image[y:y+h, x:x+w]

            keypoints = fast.detect(roi)

            #height = np.max(keypoints) - np.min(keypoints)
            #obstacle = self.Obstacle(x,y,height)
            roi_with_keypoints = cv2.drawKeypoints(roi, keypoints, None, color=(255, 0, 0))

            image[y:y+h, x:x+w] = roi_with_keypoints
        
        return image


    def process_image(self, image_path):
        # Load the image
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.get_logger().error("Could not read image.")
            return
        
        # Process the image
        cv2.imshow("Original Image", cv_image)
        segmented, boxes = self.segmentation(cv_image)
        extracted = self.feature_extraction(cv_image, boxes)
        
        cv2.imshow("Segmented Image", segmented)
        #cv2.imshow("Extracted Features", extracted)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    image_path = '/home/urc/URC/rover-ros/src/obstacle_detection/test_images/rocks1.jpg'  # Replace with your image path
    node.process_image(image_path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

