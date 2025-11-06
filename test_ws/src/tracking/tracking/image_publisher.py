# In order to get video from webcam, run this
# "C:\Program Files\VideoLAN\VLC\vlc.exe" dshow:// :dshow-vdev="ACER HD User Facing" :dshow-size=1280x720 :sout="#transcode{vcodec=MJPG,vb=4000,scale=1}:http{mux=mpjpeg,dst=:8080/video}" :no-sout-all :sout-keep
#           Replace with the name of your camera/capture device ^^^^^^^^^^^^^^^^^^^

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self, Hz=10):
        super().__init__("image_publisher")
        self.publisher = self.create_publisher(Image, "camera/image_raw", 10)
        self.create_timer(1/Hz, self.timer_callback)

        
        self.camera = "http://10.159.188.56:8080/video" # Change to your camera
        self.cap = cv2.VideoCapture(self.camera) # change this to the proper camera
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Set the buffer to 1 frame, so we constantly get new frames

        self.bridge = CvBridge()
        self.get_logger().info('Camera publisher started.')

        

    def timer_callback(self):
        for _ in range(3): # Cursed way to overcome the buffer opencv uses on a M-JPEG stream, used when ImagePublisher.Hz is a low number
            self.cap.grab()
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame")
            return

        # Convert form OpenCV image (numpy array) to Image Message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(img_msg)
        self.get_logger().info('Published Frame.')
    
    def destroy_node(self):
        #self.cap.release()
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
