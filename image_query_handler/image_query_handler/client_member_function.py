from timeit import default_timer as timer
import time

from groundlight_interfaces.srv import ImageQuery, GrabFrame
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class SampleGroundlightApp(Node):
    def __init__(self):
        super().__init__('sample_groundlight_client')

        # Create the service client for image queries
        self.iq_client = self.create_client(ImageQuery, 'image_query')
        while not self.iq_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Groundlight image query service not available, waiting again...')
        self.req = ImageQuery.Request()

        # Create the service client for grabbing frames
        self.grab_frame_client = self.create_client(GrabFrame, 'camera/grab_frame')
        while not self.grab_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera stream service not available, waiting again...')
        self.frame_req = GrabFrame.Request()

    def submit_image_query(self, image: Image, detector_id: str):
        self.req.image = image
        self.req.detector_id = detector_id
        future = self.iq_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def grab_frame(self) -> Image:
        future = self.grab_frame_client.call_async(self.frame_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().image

def main():
    rclpy.init()
    sample_groundlight_app = SampleGroundlightApp()

    # submit some image queries
    for _ in range(1):
        image_msg = sample_groundlight_app.grab_frame()
        sample_groundlight_app.get_logger().info(f'Is the black arrow aligned with the fiducial on the gear?')
        
        detector_id = 'det_2WjC9cOGczrwBWzJE8wPfpDSriG' # gear alignment
        sample_groundlight_app.get_logger().info(f'Sending request at {timer()}...')
        response = sample_groundlight_app.submit_image_query(image_msg, detector_id)
        sample_groundlight_app.get_logger().info(f'Result received! {response.result}')

    sample_groundlight_app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()