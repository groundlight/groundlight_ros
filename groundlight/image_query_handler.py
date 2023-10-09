import rclpy
from rclpy.node import Node
import time

class NewNode(Node):
    def __init__(self):
        super().__init__('image_query_handler')
        self.get_logger().info('image_query_handler node is up and running!')

        # define a camera subscriber

        # define a image_query subscriber

        # talk to Groundlight

def main(args=None):
    rclpy.init(args=args)
    node = NewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
