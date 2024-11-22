import rclpy
from rclpy.node import Node

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_node')  # Set the name of the node
        self.get_logger().info('Occupancy Grid Node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
