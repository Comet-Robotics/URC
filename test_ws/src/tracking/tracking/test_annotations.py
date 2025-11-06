import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import ImageAnnotations, TextAnnotation, PointsAnnotation, Point2
from builtin_interfaces.msg import Time
from std_msgs.msg import ColorRGBA

class ImageAnnotationNode(Node):
    def __init__(self):
        super().__init__("image_annotation_node")

        self.sub = self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)

        self.pub = self.create_publisher(ImageAnnotations, "camera/annotations", 10)

        self.get_logger().info("Waiting for images")

    def image_callback(self, msg: Image):
        self.get_logger().info("Recieved Image")

        annotations = ImageAnnotations()

        label = TextAnnotation()
        label.timestamp = msg.header.stamp
        label.position.x = 200.0
        label.position.y = 50.0
        label.text = "This is a label"
        label.font_size = 16.0
        label.text_color.r = 1.0
        label.text_color.g = 1.0
        label.text_color.b = 1.0
        label.text_color.a = 1.0

        annotations.texts.append(label)

        # --- Bounding box ---
        bbox = PointsAnnotation()
        bbox.timestamp = msg.header.stamp
        bbox.type = PointsAnnotation.LINE_LOOP
        bbox.thickness = 2.0

        
        bbox.outline_color.r = 0.0
        bbox.outline_color.g = 1.0
        bbox.outline_color.b = 0.0
        bbox.outline_color.a = 1.0

        
        bbox.points = [
            Point2(x=200.0, y=100.0),
            Point2(x=350.0, y=100.0),
            Point2(x=350.0, y=200.0),
            Point2(x=200.0, y=200.0),
        ]

        annotations.points.append(bbox)


        self.get_logger().info("Publishing annotations")
        self.pub.publish(annotations)

def main(args=None):
    rclpy.init(args=args)
    node = ImageAnnotationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()