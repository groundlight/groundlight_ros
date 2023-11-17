import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time, Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gl_interfaces.action import ImageQuery
from gl_interfaces.srv import GrabFrame

class KinovaDemo(Node):
    def __init__(self):
        super().__init__('kinova_demo')
        self._action_client = ActionClient(self, ImageQuery, '/groundlight/image_query')

        self.grab_frame_client = self.create_client(GrabFrame, '/groundlight/grab_frame')
        while not self.grab_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera stream service not available, waiting again...')
        self.frame_req = GrabFrame.Request()

        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        time.sleep(2) # wait for the publisher to be ready

        self.get_logger().info('Groundlight Kinova demo has launched.')

    def send_goal(self, 
                    image: Image, 
                    detector_id: str = '', 
                    query: str = '', 
                    name: str = '', 
                    patience_time: float = 0.0,
                    confidence_threshold: float = 0.0,
                    human_review: str = 'DEFAULT'):
        
        header = Header()
        clock = rclpy.clock.Clock()
        current_time = clock.now()
        header.stamp = Time(sec=current_time.seconds_nanoseconds()[0], nanosec=current_time.seconds_nanoseconds()[1])
        header.frame_id = 'camera_link'

        goal_msg = ImageQuery.Goal()
        goal_msg.header = header
        goal_msg.params.detector_id = detector_id
        goal_msg.params.query = query
        goal_msg.params.name = name
        goal_msg.params.patience_time = patience_time
        goal_msg.params.confidence_threshold = confidence_threshold
        goal_msg.params.human_review = human_review
        goal_msg.image = image

        self._action_client.wait_for_server(timeout_sec=1.0)

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def move_joints(self, points: list, time_from_start: int = 5):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_4', 'joint_5', 'joint_3', 'joint_6', 'joint_7', 'joint_2']
        point = JointTrajectoryPoint()
        point.positions = points
        point.time_from_start = Duration(sec=time_from_start)
        msg.points = [point]
        self.joint_trajectory_pub.publish(msg)
        time.sleep(time_from_start)

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
        self.get_logger().info(f'Result: {self.iq_msg_to_str(result)}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {self.iq_msg_to_str(feedback)}')

    def grab_frame(self) -> Image:
        future = self.grab_frame_client.call_async(self.frame_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().image
    
    def iq_msg_to_str(self, msg: ImageQuery) -> str:
        return (
            f'{msg.response.image_query_id} label={msg.response.label} '
            f'confidence={msg.response.confidence:.4f} confidence_threshold={msg.params.confidence_threshold:.4f} '
        )

def main(args=None):
    rclpy.init(args=args)
    node = KinovaDemo()

    image_query_params = {
        'query': 'Is there a gear in the minecart?',
        'name': 'Gear in Mine Cart',
        'human_review': 'NEVER',
        'patience_time': 60.0,
    }

    home_joints = [
        -0.5182688752753835,
        1.965048650914085,
        0.049327399068321356,
        -2.493916408826662,
        -0.5187722789411363,
        -8.005670007888146,
        -0.18635724237142284,
    ]
    node.move_joints(home_joints)

    joints = [
        -2.6774568410146387,
        -0.5083902137671836,
        0.6689073665879121,
        -0.3193610053362402,
        -1.8118085004623035,
        -4.873036907559572,
        -0.4376063336765131,
        ]
    node.move_joints(joints)
    time.sleep(1)
    image = node.grab_frame()
    node.send_goal(image, **image_query_params)

    joints = [
        -1.8795565311459212,
        -1.018070308782455,
        0.08671767327862559,
        0.30360852899761037,
        -1.790324146897249,
        -4.741531825028434,
        0.032179420784404875,
    ]
    node.move_joints(joints)
    time.sleep(1)
    image = node.grab_frame()
    node.send_goal(image, **image_query_params)

    joints = [
        -0.5988280584448054,
        0.4697190470483502,
        -1.1897538992191494,
        -2.1064419031864006,
        1.842244041178077,
        -7.439415679511612,
        -0.5772445179166635,
    ]
    node.move_joints(joints)
    time.sleep(1)
    image = node.grab_frame()
    node.send_goal(image, **image_query_params)

    node.move_joints(home_joints)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()