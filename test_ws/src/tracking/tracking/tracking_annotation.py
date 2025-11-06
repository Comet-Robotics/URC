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


class AnnotateNode(Node):
    def __init__(self):
        super().__init__("tracking_annotation_node")

        self.sub = self.create_subscription(Detections, "camera/detections", self.annotate, 10)
        self.pub = self.create_publisher(ImageAnnotations, "camera/annotations", 10)

    def annotate(self, msg: Detections):
        self.get_logger().info("Recieved tracking data")

        annotation = ImageAnnotations()

        for b in msg.boxes:
            x = b.x
            y = b.y
            w = b.w
            h = b.h

            bbox = PointsAnnotation()
            bbox.timestamp = msg.timestamp
            bbox.type = PointsAnnotation.LINE_LOOP
            bbox.thickness = 2.0

            
            bbox.outline_color.r = 1.0
            bbox.outline_color.g = 0.0
            bbox.outline_color.b = 0.0
            bbox.outline_color.a = 1.0

            bbox.points = [
                Point2(x=float(x), y=float(y)),
                Point2(x=float(x+w), y=float(y)),
                Point2(x=float(x+w), y=float(y+h)),
                Point2(x=float(x), y=float(y+h))                
            ]
            
            annotation.points.append(bbox)

        self.get_logger().info("Publishing annotations")
        self.pub.publish(annotation)

def main(args=None):
    rclpy.init(args=args)
    node = AnnotateNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == '__main__':
    main()