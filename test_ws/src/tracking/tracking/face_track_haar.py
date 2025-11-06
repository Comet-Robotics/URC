import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import ImageAnnotations, TextAnnotation, PointsAnnotation, Point2
from builtin_interfaces.msg import Time
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import numpy as np

from custom_interfaces.msg import BoundingBox, Detections
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('tracking')

class TrackingNode(Node):
    def __init__(self):
        super().__init__("face_track_haar")
        # Haar cascade classifier. 
        self.face_cascade = cv2.CascadeClassifier(os.path.join(pkg_share, 'models', 'haarcascade_frontalface_default.xml'))

        self.sub = self.create_subscription(Image, "camera/image_raw", self.detect_face, 10)
        self.pub = self.create_publisher(Detections, "camera/detections", 10)

        self.bridge = CvBridge()
        
        self.get_logger().info("Waiting for images")


    def detect_face(self, msg):
        self.get_logger().info("Recieved Image")

        img = self.bridge.imgmsg_to_cv2(msg)
        face_img = img.copy()
        face_rect = self.face_cascade.detectMultiScale(face_img, scaleFactor=1.2, minNeighbors=5)

        detections = Detections()

        detections.boxes = []
        
        self.get_logger().info(f"Num faces detected: {len(face_rect)}")

        # Bounding boxes
        for (x, y, w, h) in face_rect:
            bbox = BoundingBox()
            bbox.x = int(x)
            bbox.y = int(y)
            bbox.w = int(w)
            bbox.h = int(h)

            detections.boxes.append(bbox)

        # detections.timestamp = TimeMsg()
        # detections.timestamp.sec = msg.header.stamp.sec
        # detections.timestamp.nanosec = msg.header.stamp.nanosec

        detections.timestamp = msg.header.stamp

        self.get_logger().info(f"\nBoxes: {len(detections.boxes)}\n")
        for b in detections.boxes:
            self.get_logger().info("\nBox:")
            self.get_logger().info(f"x: {b.x}; {type(b.x)}, y: {b.y}; {type(b.y)}, w: {b.w}; {type(b.w)}, h: {b.h}; {type(b.h)}\n")

        self.get_logger().info(f"\nDetections:\n stamp:\ntype: {type(detections.timestamp)}, sec: {detections.timestamp.sec}, nanosec: {detections.timestamp.nanosec}\n")
        self.get_logger().info(f"\nMessage:\n stamp:\ntype: {type(msg.header.stamp)}, sec: {msg.header.stamp.sec}, nanosec: {msg.header.stamp.nanosec}\n")
        
        self.get_logger().info("Publishing detections")
        self.pub.publish(detections)

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()