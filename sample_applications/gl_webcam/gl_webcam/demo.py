import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node, ParameterDescriptor
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from gl_interfaces.action import ImageQuery
from gl_interfaces.srv import GrabFrame

import datetime

class WebcamExample(Node):
    def __init__(self):
        super().__init__('webcam_example', namespace='groundlight')
        descriptor = ParameterDescriptor(
            description='Query for the image processing',
            dynamic_typing=True
        )
        self.query = self.declare_parameter('query', None, descriptor=descriptor).value
        if self.query is None:
            self.get_logger().error(
                'Please provide a query. Example: ros2 run gl_webcam demo --ros-args -p query:="Is the person giving a thumbs up?"'
                )
            rclpy.shutdown()
            exit()
        else:
            self.get_logger().info(f'Creating a new detector for query: "{self.query}"')

        self._action_client = ActionClient(self, ImageQuery, '/groundlight/image_query')

        self.grab_frame_client = self.create_client(GrabFrame, '/groundlight/grab_frame')
        while not self.grab_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera stream service not available, waiting again...')
        self.frame_req = GrabFrame.Request()

        self.start_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Groundlight action server.')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'The answer is {result.response.label} with a confidence of {result.response.confidence:.4f}')

    def feedback_callback(self, feedback_msg):
        # Access the response through the feedback attribute
        response = feedback_msg.feedback.response
        label = response.label
        confidence = response.confidence
        self.get_logger().info(
            f'Working on getting you an answer. Provisional answer is {label} with a confidence of {confidence:.4f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = WebcamExample()

    if not node._action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().error('Groundlight action server not available.')
        rclpy.shutdown()
        return
    
    while True:
        user_input = input(
            'Press any key to capture and submit your image query. Press `q` to quit. '
            )
        if user_input.lower() == 'q':
            break

        # Grab a frame 
        future = node.grab_frame_client.call_async(node.frame_req)
        rclpy.spin_until_future_complete(node, future)
        image = future.result().image

        # Submit the image query to Groundlight
        header = Header()
        current_time = node.get_clock().now()
        header.stamp = Time(sec=current_time.seconds_nanoseconds()[0], nanosec=current_time.seconds_nanoseconds()[1])
        goal_msg = ImageQuery.Goal()
        goal_msg.header = header
        goal_msg.params.query = node.query
        goal_msg.params.name = f"groundlight_ros_demo_{node.start_time}"
        goal_msg.params.wait = 30.0
        goal_msg.params.human_review = "DEFAULT"
        goal_msg.params.confidence_threshold = 0.6

        goal_msg.image = image

        node.get_logger().info(
            f'Submitting image query to Groundlight: {goal_msg.params.query}'
            )
        future = node._action_client.send_goal_async(goal_msg, feedback_callback=node.feedback_callback)
        future.add_done_callback(node.goal_response_callback)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)

    node.get_logger().info('Ending Groundlight webcam demo.')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()