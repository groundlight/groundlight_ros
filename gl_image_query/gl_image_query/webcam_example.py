import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from gl_interfaces.action import ImageQuery
from gl_interfaces.srv import GrabFrame

class WebcamExample(Node):
    def __init__(self):
        super().__init__('webcam_example', namespace='groundlight')
        self._action_client = ActionClient(self, ImageQuery, '/groundlight/image_query')

        self.grab_frame_client = self.create_client(GrabFrame, '/groundlight/grab_frame')
        while not self.grab_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera stream service not available, waiting again...')
        self.frame_req = GrabFrame.Request()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Groundlight action server.')
            return
        else:
            self.get_logger().info('Goal accepted by Groundlight action server.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        pass

    def feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = WebcamExample()

    if not node._action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().error('Groundlight action server not available.')
        rclpy.shutdown()
        return
    
    while True:
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
        goal_msg.params.query = "Is there a part in the vise?"
        goal_msg.params.name = "part_in_vise"
        # goal_msg.params.detector_id = 'det_xxxxxxxxx' # Alternatively, you can refer to your detector by ID
        goal_msg.params.wait = 30.0
        goal_msg.params.human_review = "DEFAULT"
        goal_msg.image = image

        node.get_logger().info(f'Submitting image query to Groundlight: {goal_msg.params.query}')
        future = node._action_client.send_goal_async(goal_msg, feedback_callback=node.feedback_callback)
        future.add_done_callback(node.goal_response_callback)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)

        if input('Would you like submit another image query? (y/n) ') != 'y':
            break
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()