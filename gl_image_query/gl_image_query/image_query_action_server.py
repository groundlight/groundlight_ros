import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from cv_bridge import CvBridge

from gl_interfaces.action import ImageQuery
from gl_interfaces.msg import ImageQueryData

from groundlight import Groundlight

bridge = CvBridge()

gl = Groundlight() # Connect to Groundlight

TIMEOUT_SEC = 120 
POLLING_FREQ_SEC = .25

class IQActionServer(Node):
    def __init__(self):
        super().__init__('image_query_action_server')
        self._action_server = ActionServer(
            self,
            ImageQuery,
            '/groundlight/image_query',
            self.execute_callback)
        
        # Publish the feedback and result on separate topics so that other nodes can subscribe
        self.feedback_pub = self.create_publisher(ImageQueryData, '/image_query/feedback', 10)
        self.result_pub = self.create_publisher(ImageQueryData, '/image_query/result', 10)

        self.get_logger().info('Groundlight image query action server is online.')

    def execute_callback(self, goal_handle):
        # Convert image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(goal_handle.request.image, desired_encoding='bgr8')

        # Get detector
        det = gl.get_detector(id=goal_handle.request.detector_id)

        # Submit image query
        iq = gl.ask_async(det, image=cv_image)
        self.get_logger().info(f'Processing image query: {iq.query}')

        # Initialize feedback message
        feedback_msg = ImageQuery.Feedback()
        feedback_msg.data.id = iq.id
        feedback_msg.data.query = iq.query

        # Poll image query
        time_waited_sec = 0.0
        while True:
            # Get the image query
            iq = gl.get_image_query(iq.id)
            confidence = iq.result.confidence if iq.result.confidence is not None else 1.0
            label = iq.result.label.value

            # Publish feedback
            self.get_logger().info(f'Feedback: confidence={confidence} label={label}')
            feedback_msg.data.confidence = confidence
            feedback_msg.data.label = label
            goal_handle.publish_feedback(feedback_msg)
            self.feedback_pub.publish(feedback_msg.data)
            
            # Check if answer is final
            answer_is_final = confidence is None or confidence >= det.confidence_threshold
            if answer_is_final:
                goal_handle.succeed()
                result = ImageQuery.Result()
                result.data.id = feedback_msg.data.id
                result.data.query = feedback_msg.data.query
                result.data.label = feedback_msg.data.label
                result.data.confidence = feedback_msg.data.confidence
                self.get_logger().info(f'Result: confidence={confidence} label={label}')
                break
            
            # Check for timeout
            time.sleep(POLLING_FREQ_SEC)
            time_waited_sec += POLLING_FREQ_SEC
            if time_waited_sec > TIMEOUT_SEC:
                self.get_logger().error(f'Timed out waiting for answer from Groundlight on detector {det.id}')
                result = ImageQuery.Result()
                break
        
        self.result_pub.publish(feedback_msg.data)
        return result

def main(args=None):
    rclpy.init(args=args)
    iq_action_server = IQActionServer()
    rclpy.spin(iq_action_server)

if __name__ == '__main__':
    main()