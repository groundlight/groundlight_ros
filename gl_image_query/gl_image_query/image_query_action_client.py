import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import Image

from gl_interfaces.action import ImageQuery
from gl_interfaces.srv import GrabFrame

class IQActionClient(Node):
    def __init__(self):
        super().__init__('image_query_action_client')
        self._action_client = ActionClient(self, ImageQuery, 'image_query')

        # Create the service client for grabbing frames
        self.grab_frame_client = self.create_client(GrabFrame, 'camera/grab_frame')
        while not self.grab_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera stream service not available, waiting again...')
        self.frame_req = GrabFrame.Request()

        self.get_logger().info('Sample Groundlight image query action client has launched.')

    def send_goal(self, image: Image, detector_id: str):
        goal_msg = ImageQuery.Goal()
        goal_msg.image = image
        goal_msg.detector_id = detector_id
        goal_msg.wait = 0.0
        goal_msg.human_review = 'DEFAULT'
        goal_msg.inspection_id = ''

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback}')

    def grab_frame(self) -> Image:
        future = self.grab_frame_client.call_async(self.frame_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().image

def main(args=None):
    rclpy.init(args=args)
    action_client = IQActionClient()
    image = action_client.grab_frame()
    detector_id = 'det_2WjC9cOGczrwBWzJE8wPfpDSriG' # gear alignment
    action_client.send_goal(image, detector_id)

    action_client.get_logger().info('Move the robot!')
    time.sleep(10)

    action_client.get_logger().info('Submitting another image query!')
    image = action_client.grab_frame()
    detector_id = 'det_2WgB8XzLLEq89cvPbwep1nhIOTL' # person detector
    action_client.send_goal(image, detector_id)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()