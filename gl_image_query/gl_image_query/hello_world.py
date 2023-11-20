import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from gl_interfaces.action import ImageQuery
from gl_interfaces.srv import GrabFrame

class HelloWorld(Node):
    def __init__(self):
        super().__init__('hello_world_groundlight')
        self._action_client = ActionClient(self, ImageQuery, '/groundlight/image_query')

        self.grab_frame_client = self.create_client(GrabFrame, '/groundlight/grab_frame')
        while not self.grab_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera stream service not available, waiting again...')
        self.frame_req = GrabFrame.Request()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        else:
            self.get_logger().info('Goal accepted.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'The final answer is {result.response.label} with a confidence of {result.response.confidence:.4f}.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Waiting for a confident answer. Provisional answer is {feedback.response.label} with a confidence of {feedback.response.confidence:.4f}.')
    
    def iq_msg_to_str(self, msg: ImageQuery) -> str:
        return (
            f'{msg.response.image_query_id} label={msg.response.label} '
            f'confidence={msg.response.confidence:.4f} confidence_threshold={msg.params.confidence_threshold:.4f} '
        )

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorld()

    if not node._action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().error('Groundlight action server not available.')
        rclpy.shutdown()
        return
    
    node.get_logger().info('Groundlight ROS demo has started. Your robot will now take a picture and ask Groundlight if there is a person in the image.')
    time.sleep(2)
    countdown = 3
    while countdown > 0:
        node.get_logger().info(f'{countdown}...')
        countdown -= 1
        time.sleep(1)

    # Grab a frame 
    future = node.grab_frame_client.call_async(node.frame_req)
    rclpy.spin_until_future_complete(node, future)
    image = future.result().image

    # Submit the image query to Groundlight
    header = Header()
    clock = rclpy.clock.Clock()
    current_time = clock.now()
    header.stamp = Time(sec=current_time.seconds_nanoseconds()[0], nanosec=current_time.seconds_nanoseconds()[1])
    header.frame_id = 'camera_color_frame' # Optional: include the frame from which the image was taken. Used to generate RViz markers.
    goal_msg = ImageQuery.Goal()
    goal_msg.header = header
    goal_msg.params.query = "Is there a person visible?"
    goal_msg.params.name = "groundlight_ros_hello_world"
    # goal_msg.params.detector_id = 'det_xxxxxxxxx' # Alternatively, you can refer to your detector by ID
    goal_msg.params.patience_time = 30.0
    goal_msg.params.confidence_threshold = .75
    goal_msg.params.human_review = "DEFAULT"
    goal_msg.image = image

    node.future = node._action_client.send_goal_async(goal_msg, feedback_callback=node.feedback_callback)
    node.future.add_done_callback(node.goal_response_callback)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()