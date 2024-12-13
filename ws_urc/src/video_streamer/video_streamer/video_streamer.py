import subprocess

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse
import shlex

def launch_transcoder(ip_address,port=5000, fps=30):

    ffmpeg_cmd = f"ffmpeg -y -f rawvideo -pixel_format bgr24 -video_size 640x480 -framerate {fps} -i - " \
                     "-vcodec libvpx -vf \"drawtext=:text=\'%{localtime}\':x=10:y=10:fontsize=24:fontcolor=white\" -deadline 1 -g 10 -error-resilient 1 -auto-alt-ref 1 " \
                      f"-f rtp rtp://{ip_address}:{port} -sdp_file stream.sdp" 
               

        # Launch FFmpeg subprocess
    return subprocess.Popen(shlex.split(ffmpeg_cmd), stdin=subprocess.PIPE,stderr=subprocess.PIPE)


# Example usage

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('video_streamer')
      # Declare parameters with a default value
        self.declare_parameter('ip', '100.100.246.41')
        
        # Retrieve parameters
        self.ip_address = self.get_parameter('ip').get_parameter_value().string_value
        
        self.rgb_transcoder = launch_transcoder(self.ip_address)
        self.depth_transcoder = launch_transcoder(self.ip_address,port=5001)
        
        # Create a subscriber for the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # The topic to which your RealSense publishes images
            self.image_callback,
            10)

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',  # The topic to which your RealSense publishes images
            self.image_depth_callback,
            10)
        
        # Bridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
 
        try:
            self.rgb_transcoder.stdin.write(frame.tobytes())
        except BrokenPipeError:
            self.get_logger().error("FFmpeg process has terminated.")
            self.rgb_transcoder = launch_transcoder(self.ip_address)
            rclpy.shutdown()

    def image_depth_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        normalized_depth = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
        normalized_depth = np.uint8(normalized_depth)  # Convert to 8-bit

        # Convert the grayscale 8-bit depth image to BGR format
        bgr_depth_image = cv2.cvtColor(normalized_depth, cv2.COLOR_GRAY2BGR)


 
        try:
            self.depth_transcoder.stdin.write(bgr_depth_image.tobytes())
        except BrokenPipeError:
            self.get_logger().error("FFmpeg process has terminated.")
            self.depth_transcoder = launch_transcoder(self.ip_address,port=5001)

            rclpy.shutdown()

def main(args=None):


    rclpy.init(args=args)

    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


