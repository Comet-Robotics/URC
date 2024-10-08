import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse
import shlex

def launch_transcoder(ip_address,port=5000, fps=30):
    ffmpeg_cmd = f"ffmpeg -y -f rawvideo -pix_fmt rgb24 -video_size 640x480 -framerate {fps} -i pipe:0 " \
                     "-c:v libx264 -vf \"drawtext=:text=\'%{localtime}\':x=10:y=10:fontsize=24:fontcolor=white\" -preset ultrafast -tune zerolatency " \
                      f"-f rtp rtp://{ip_address}:{port} -sdp_file stream.sdp" 
               

        # Launch FFmpeg subprocess
    return subprocess.Popen(shlex.split(ffmpeg_cmd), stdin=subprocess.PIPE)


# Example usage

class ImageSubscriber(Node):
    def __init__(self,ip_address):
        super().__init__('video_streamer')

        self.rgb_transcoder = launch_transcoder(ip_address)
        
        # Create a subscriber for the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # The topic to which your RealSense publishes images
            self.image_callback,
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
            rclpy.shutdown()

def main(args=None):

  

    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="ROS 2 RealSense Image Subscriber")
    parser.add_argument('--ip', type=str, help='IP address of the camera', required=True)
    args = parser.parse_args()
    node = ImageSubscriber(ip_address=args.ip)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


