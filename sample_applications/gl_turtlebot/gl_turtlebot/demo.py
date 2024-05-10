import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import NavigateToPose

from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from gl_interfaces.srv import GrabFrame
from gl_interfaces.action import ImageQuery

class TurtlebotDemo(Node):
    def __init__(self):
        super().__init__('turtlebot_demo', namespace='groundlight')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.camera_client = self.create_client(GrabFrame, '/groundlight/grab_frame')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /groundlight/grab_frame not available, waiting again...')
        self.grab_frame_req = GrabFrame.Request()

        self.iq_action_client = ActionClient(self, ImageQuery, '/groundlight/image_query')

    def navigate(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        goal_msg.pose.header.frame_id = 'map'

        # Ensure that the action server is available
        self.nav_client.wait_for_server()

        # Send the goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_response_callback)

        # Wait for the server to accept the goal
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            return None

        # Get the result of the action
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result
    
    def navigation_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Navigation goal reached')
        else:
            self.get_logger().info('Navigation goal failed')

    def grab_frame(self):
        future = self.camera_client.call_async(self.grab_frame_req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info('Received image timestamp: %s' % response.image.header.stamp)
                return response.image
            else:
                self.get_logger().error('Received no response from service')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
            return None

    def submit_image_query(self, detector_name: str, detector_query: str):
        img_msg = self.grab_frame()
        self.get_logger().info(f'Got frame {img_msg.header.stamp}')
        self.get_logger().info(f"Submitting image query '{detector_query}...'")

        goal_msg = ImageQuery.Goal()
        goal_msg.header = img_msg.header
        goal_msg.params.name = detector_name
        goal_msg.params.query = detector_query
        goal_msg.image = img_msg

        if not self.iq_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Image Query Action Server not available!')
            return

        future = self.iq_action_client.send_goal_async(goal_msg, feedback_callback=self.iq_feedback_callback)
        future.add_done_callback(self.iq_response_callback)

    def iq_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('Got feedback')

    def iq_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Image query goal rejected.')
            return
        else:
            self.get_logger().info('Image query goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.iq_result_callback)

    def iq_result_callback(self, future):
        self.get_logger().info('Got result!')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotDemo()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    UP_ORIENTATION = (0.0, 0.0, -0.707, 0.707)
    DOWN_ORIENTATION = (0.0, 0.0, 0.707, 0.707)
    AISLE_DET = ('aisle_clear', 'Is the aisle clear of obstructions?')
    TRUCK_DET = ('truck_outside', 'Is there a truck waiting outside? ')

    while True:
        goals = [
            (node.navigate, (-5.841587543487549, -3.261117696762085, 0.0, *UP_ORIENTATION)), # aisle 3 bottom
            (node.submit_image_query, AISLE_DET),
            (node.navigate, (-14.006568822464489, -2.510155155879514, 0.0, *UP_ORIENTATION)), # aisle 4 bottom
            (node.submit_image_query, AISLE_DET),
            (node.navigate, (-12.283817291259766, -23.275110244750977, 0.0, *DOWN_ORIENTATION)), # aisle 4 top 
            (node.submit_image_query, AISLE_DET), 
            (node.navigate, (-5.551569938659668, -23.304880142211914, 0.0, *DOWN_ORIENTATION)), # aisle 3 top 
            (node.submit_image_query, AISLE_DET), 
            (node.navigate, (2.449106454849243, -23.02561378479004, 0.0, *DOWN_ORIENTATION)), # aisle 2 top
            (node.submit_image_query, AISLE_DET),
            (node.navigate, (4.599254944774189, -24.610295710617095, 0.0, 0.0, 0.0, -0.2715895473737527, 0.9624131741395273)), # looking at truck
            (node.submit_image_query, TRUCK_DET),
            (node.navigate, (9.518928527832031, -22.613218307495117, 0.0, *DOWN_ORIENTATION)), # aisle 1 top
            (node.submit_image_query, AISLE_DET),
            (node.navigate, (10.210944175720215, -3.231522798538208, 0.0, *UP_ORIENTATION)), # aisle 1 bottom
            (node.submit_image_query, AISLE_DET),
            (node.navigate, (1.7633006572723389, -3.7294914722442627, 0.0, *UP_ORIENTATION)), # aisle 2 bottom 
            (node.submit_image_query, AISLE_DET),
            (node.navigate, (-0.18684534152910143, 1.4266437858826522, 0.0, 0.0, 0.0, -0.64511168236156, 0.7640846528005732)), # home
        ]

        for goal_func, params in goals:
            result = goal_func(*params)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
