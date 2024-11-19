import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist

class ArucoChase(Node):
    turn_speed = 8.0
    forward_speed = 2.0


    depth_image = None



    def __init__(self):
        super().__init__('aruco_chase')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  
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

        self.velocity_msg = Twist()

        self.bridge = CvBridge()

    def color_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
       
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()

        # Create the ArUco detector
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray)
        # Print the detected markers
        print("Detected markers:", ids)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
         


            first = corners[0]

            center_x = np.mean([point[0] for point in first[0]])
            center_y = np.mean([point[1] for point in first[0]])
            center = (int(center_x), int(center_y))
            height, width, channels = cv_image.shape

            feet = self.depth_image[int(center_y),int(center_x)]/304.8

            if (feet > 4):
                self.velocity_msg.linear.x = -self.forward_speed
            else:
                self.velocity_msg.linear.x = 0.0
            delta = center[0] - (width/2)
            print(delta)
            if (abs(delta) < 15):
                self.velocity_msg.angular.z = 0.0
            elif (delta > 0):
                self.velocity_msg.angular.z = max(self.turn_speed * (2 * abs(delta)/width),2.0)
            elif (delta < 0):
                self.velocity_msg.angular.z = -max(self.turn_speed* (2 * abs(delta)/width),2.0)



        else:
            self.velocity_msg.linear.x = 0.0  # Set forward velocity (m/s)
            self.velocity_msg.angular.z = 0.0  # Set rotational velocity (rad/s)

        cv2.imshow('Detected Markers', cv_image)
        cv2.waitKey(1)
        # Publish the message to /cmd_vel
        self.publisher_.publish(self.velocity_msg)
        self.get_logger().info(f"Publishing: Linear X: {self.velocity_msg.linear.x}, Angular Z: {self.velocity_msg.angular.z}")

    def depth_callback(self, msg):
        # Convert depth image to CV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        #cv2.imshow("depth image", depth_image)   
   




def main(args=None):
    rclpy.init(args=args)
    node = ArucoChase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

