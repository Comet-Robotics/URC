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
    k_size_blurring = 5
    k_size_opening = 3
    k_size_closing = 5
    k_size_dilation = 3
    block_size = 7
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
            '/camera/color/image_raw',
            self.color_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        cv2.namedWindow('Image Segmentation')
        self.create_trackbars()

        self.bridge = CvBridge()
    def create_trackbars(self):
        """Create trackbars for adjusting parameters."""
        def nothing(x):
            print(x)
        
        cv2.createTrackbar('Blur Size', 'Image Segmentation', self.k_size_blurring, 21, nothing)
        cv2.createTrackbar('Opening Size', 'Image Segmentation', self.k_size_opening, 21, nothing)
        cv2.createTrackbar('Closing Size', 'Image Segmentation', self.k_size_closing, 21, nothing)
        cv2.createTrackbar('Dilation Size', 'Image Segmentation', self.k_size_dilation, 21, nothing)
        cv2.createTrackbar('Block Size', 'Image Segmentation', self.block_size, 51, nothing)
        cv2.createTrackbar('Gauss Threshold', 'Image Segmentation', self.gauss_threshold, 10, nothing)
    def color_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        segmented, boxes = self.segmentation(cv_image)
        cv2.imshow("segmented features", segmented)
        extracted = self.feature_extraction(cv_image, boxes)
        cv2.imshow("extracted features", extracted)

        cv2.waitKey(2)

    def depth_callback(self, msg):
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
            depth_image_colored = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)


            segmented, boxes = self.segmentation(depth_image_colored)
            cv2.imshow("Depth Extracted Segmented", segmented)
            extracted = self.feature_extraction(depth_image_colored, boxes)
            # Display the depth image
            cv2.imshow('Depth Extracted Features', extracted)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def segmentation(self, image):
        self.k_size_blurring = cv2.getTrackbarPos('Blur Size', 'Image Segmentation') * 2 + 1
        self.k_size_opening = cv2.getTrackbarPos('Opening Size', 'Image Segmentation')
        self.k_size_closing = cv2.getTrackbarPos('Closing Size', 'Image Segmentation')
        self.k_size_dilation = cv2.getTrackbarPos('Dilation Size', 'Image Segmentation')
        self.block_size = cv2.getTrackbarPos('Block Size', 'Image Segmentation') * 2 + 1
        self.gauss_threshold = cv2.getTrackbarPos('Gauss Threshold', 'Image Segmentation')

        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) #rgb -> hsv conversion

        final_img = np.zeros_like(image) #initialize final image

        #initialize kernels
        opening_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size_opening, self.k_size_opening))
        closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size_closing, self.k_size_closing))
        dilation_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_size_dilation, self.k_size_dilation))

        for channel in range(3):
            blurred = cv2.medianBlur(hsv_img[:, :, channel], self.k_size_blurring)

            closed = cv2.morphologyEx(blurred, cv2.MORPH_CLOSE, closing_kernel)

            opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, opening_kernel)

            thresh = cv2.adaptiveThreshold(opened, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, blockSize=self.block_size, C=self.gauss_threshold)

            final_img[thresh == 255] = 255 
        
        final_opened = cv2.morphologyEx(final_img, cv2.MORPH_OPEN, opening_kernel)
        final_dilated = cv2.morphologyEx(final_opened, cv2.MORPH_DILATE, dilation_kernel)
        final_grayscale = cv2.cvtColor(final_dilated, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Gray Scale" ,final_grayscale)

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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
