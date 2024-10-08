import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from gl_interfaces.srv import GrabFrame

class CameraService(Node):
    def __init__(self):
        super().__init__('camera_server', namespace='groundlight')

        self.declare_parameter('camera_topic', '')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        if not self.camera_topic:
            self.get_logger().warning(
                "Groundlight's camera service will not launch because no camera topic was provided." 
                "If you plan to provide your own images, you can disregard this warning. Otherwise, "
                "please provide a camera_topic argument."
                )
            rclpy.shutdown()
            exit(0)

        self.get_logger().info(f'Listening to topic {self.camera_topic}...')

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)

        self.srv = self.create_service(GrabFrame, '/groundlight/grab_frame', self.grab_frame)

        self.image_msg = None

    def image_callback(self, msg):
        if self.image_msg is None:
            self.get_logger().info(f'Receiving images from {self.camera_topic}!')
        self.image_msg = msg

    def grab_frame(self, request, response):
        if self.image_msg is None:
            self.get_logger().error(f'Unable to grab frame. No images have been received from {self.camera_topic}.')
            return response
        
        self.get_logger().info(f'Serving image from {self.camera_topic}.')
        response.image = self.image_msg
        return response

    def on_shutdown(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()