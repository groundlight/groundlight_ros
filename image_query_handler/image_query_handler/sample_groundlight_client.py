import rclpy
from rclpy.node import Node

import time

from moveit2 import MoveItCommander
from moveit2 import roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose

from groundlight_interfaces.srv import ImageQuery, GrabFrame

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

    def submit_image_query(self, 
                           image: Image, 
                           detector_id: str = None,
                           wait: float = 0.00,
                           human_review: str = 'DEFAULT',
                           inspection_id: str = '') -> object:
        """Submits an image query to Groundlight
        """
        self.req.image = image
        self.req.detector_id = detector_id
        self.req.wait = wait
        self.req.human_review = human_review
        self.req.inspection_id = inspection_id

        future = self.iq_client.call_async(self.req)

        return future
    
    def grab_frame(self) -> Image:
        future = self.grab_frame_client.call_async(self.frame_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().image


def main():
    rclpy.init()
    sample_groundlight_app = SampleGroundlightApp()

    detector_ids = (
    'det_2WjC9cOGczrwBWzJE8wPfpDSriG', # gear alignment
    'det_2WgB8XzLLEq89cvPbwep1nhIOTL' # person detector
    )

    # submit some image queries
    futures = []
    for idx, detector_id in enumerate(detector_ids):
        for _ in range(3):
            image_msg = sample_groundlight_app.grab_frame()
            future = sample_groundlight_app.submit_image_query(image_msg, detector_id)
            sample_groundlight_app.get_logger().info(f'Sending request for {detector_id}...')
            futures.append(future)

        if idx + 1 != len(detector_ids):
            print('Move the robot arm!')
            time.sleep(10)

    # wait for all the results
    for future in futures:
        rclpy.spin_until_future_complete(sample_groundlight_app, future)
        result = future.result()
        sample_groundlight_app.get_logger().info(
            f'Result received! label: {result}'
        )

    sample_groundlight_app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()