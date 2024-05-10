import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from gl_interfaces.action import ImageQuery
from gl_interfaces.srv import GrabFrame

class URDemo(Node):
    def __init__(self):
        super().__init__('ur_demo')
        self._action_client = ActionClient(self, ImageQuery, '/groundlight/image_query')

        self.grab_frame_client = self.create_client(GrabFrame, '/groundlight/grab_frame')
        while not self.grab_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera stream service not available, waiting again...')
        self.frame_req = GrabFrame.Request()

        self.script_publisher_ = self.create_publisher(String, '/urscript_interface/script_command', 10)

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_cbk,
            10)
        
        self.robot_is_moving = False

        time.sleep(2) # wait for the publisher to be ready
        self.get_logger().info('Groundlight Universal Robots demo has launched.')

    def joint_state_cbk(self, msg):
        """
        Check if the robot is currently moving based on joint velocities
        """
        velocity_thresh = .01
        for i in msg.velocity:
            if abs(i) > velocity_thresh:
                self.robot_is_moving = True
                break
        else:
            self.robot_is_moving = False

    def send_image_query(self, 
                    image: Image, 
                    detector_id: str = '', 
                    query: str = '', 
                    name: str = '', 
                    wait: float = 0.0,
                    confidence_threshold: float = 0.0,
                    human_review: str = 'DEFAULT'):
        
        header = Header()
        clock = rclpy.clock.Clock()
        current_time = clock.now()
        header.stamp = Time(sec=current_time.seconds_nanoseconds()[0], nanosec=current_time.seconds_nanoseconds()[1])
        header.frame_id = 'tool0'

        goal_msg = ImageQuery.Goal()
        goal_msg.header = header
        goal_msg.params.detector_id = detector_id
        goal_msg.params.query = query
        goal_msg.params.name = name
        goal_msg.params.wait = wait
        goal_msg.params.confidence_threshold = confidence_threshold
        goal_msg.params.human_review = human_review
        goal_msg.image = image

        self._action_client.wait_for_server(timeout_sec=1.0)

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def movej(self, pose: list):
        msg = String()
        msg.data = (
            "def my_prog():\n"
            f"  movej(p[{pose[0]}, {pose[1]}, {pose[2]}, {pose[3]}, {pose[4]}, {pose[5]}], a=1.2, v=1.00, r=0)\n"
            "end"
        )
        self.script_publisher_.publish(msg)
        self.get_logger().info('Publishing URScript: "%s"' % msg.data)

        # Wait for the robot to stop move before returning from this function
        time.sleep(.5) # pause a bit to let the robot begin moving
        self.robot_is_moving = True # Assume it is moving unless proven otherwise
        while rclpy.ok() and self.robot_is_moving:
            rclpy.spin_once(self)

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
    node = URDemo()

    person_pose = [
        .43924,
        -.00335,
        .413,
        2.236,
        -0.196,
        2.000,
    ]

    person_params = {
        'detector_id': 'det_2WgB8XzLLEq89cvPbwep1nhIOTL',
        'human_review': 'DEFAULT',
        'wait': 60.0,
    }


    gear_pose = [
        .23603,
        .04801,
        .23636,
        2.89,
        0.138,
        -0.639,
    ]

    gear_params = {
        'detector_id': 'det_2WjC9cOGczrwBWzJE8wPfpDSriG',
        'human_review': 'DEFAULT',
        'wait': 60.0,
    }

    bolts_pose = [
        -.00847,
        .25292,
        .19729,
        1.006,
        2.870,
        0.026,
    ]

    bolts_params = {
        'detector_id': 'det_2SoJ3PDnpa3c4L5J9HnlEmWmfif',
        'human_review': 'DEFAULT',
        'wait': 60.0,
    }

    base_plate_pose = [
        -.19607,
        .39732,
        .29930,
        1.006,
        2.870,
        0.026,
    ]

    base_plate_params = {
        'detector_id': 'det_2XVOHkt6URk8B7oWthynNW3yc7X',
        'human_review': 'DEFAULT',
        'wait': 60.0,
    }


    node.movej(person_pose)
    image = node.grab_frame()
    node.send_image_query(image, **person_params)

    node.movej(gear_pose)
    image = node.grab_frame()
    node.send_image_query(image, **gear_params)

    node.movej(bolts_pose)
    image = node.grab_frame()
    node.send_image_query(image, **bolts_params)


    node.movej(base_plate_pose)
    image = node.grab_frame()
    node.send_image_query(image, **base_plate_params)

    node.movej(person_pose)

    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()