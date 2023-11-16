import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from cv_bridge import CvBridge

from gl_interfaces.action import ImageQuery
from gl_interfaces.msg import ImageQueryRequest, ImageQueryFeedback, ImageQueryResult

from groundlight import Groundlight

bridge = CvBridge()

gl = Groundlight() # Connect to Groundlight

TIMEOUT_SEC = 120 
POLLING_PERIOD_SEC = .25

class IQActionServer(Node):
    def __init__(self):
        super().__init__('groundlight')
        self._action_server = ActionServer(
            self,
            ImageQuery,
            '/groundlight/image_query',
            self.execute_callback)
        
        # Republish request, feedback and result on separate topics so that other nodes can subscribe
        self.request_pub = self.create_publisher(ImageQueryRequest, '/groundlight/request', 10)
        self.feedback_pub = self.create_publisher(ImageQueryFeedback, '/groundlight/feedback', 10)
        self.result_pub = self.create_publisher(ImageQueryResult, '/groundlight/result', 10)

        self.get_logger().info('Groundlight image query action server is online.')

    def execute_callback(self, goal_handle):
        # Convert image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(goal_handle.request.image, desired_encoding='bgr8')

        # Get detector
        if goal_handle.request.params.detector_id:
            det = gl.get_detector(goal_handle.request.params.detector_id)
        elif goal_handle.request.params.name and goal_handle.request.params.query:
            det = gl.get_or_create_detector(
                name=goal_handle.request.params.name, 
                query=goal_handle.request.params.query, 
                )
        else:
            self.get_logger().error(
                'To submit an image query, you must specify either detector_id or name and query'
                )
            goal_handle.abort()
            return ImageQuery.Result()
        
        # Assign default values as needed
        # ROS messages do not support the notion of default values. For example, if a user does not provide a 
        # value for a float field, the value will be set to 0.0. Because of this, we need to assign defaults here.
        patience_time = 30.0 if goal_handle.request.params.patience_time == 0.0 else goal_handle.request.params.patience_time
        confidence_threshold = det.confidence_threshold if goal_handle.request.params.confidence_threshold == 0.0 else goal_handle.request.params.confidence_threshold
        human_review = 'DEFAULT' if goal_handle.request.params.human_review == '' else goal_handle.request.params.human_review

        # Submit image query
        iq = gl.ask_async(det, 
                          image=cv_image, 
                          patience_time=patience_time,
                          confidence_threshold=confidence_threshold,
                          human_review=human_review)
        self.get_logger().info(f'Processing image query {iq.query}...')

        # Republish request message
        request_msg = ImageQueryRequest()
        request_msg.header = goal_handle.request.header
        request_msg.params.detector_id = det.id
        request_msg.params.query = det.query
        request_msg.params.name = det.name
        request_msg.params.patience_time = patience_time
        request_msg.params.confidence_threshold = confidence_threshold
        request_msg.params.human_review = human_review
        request_msg.response.image_query_id = iq.id
        request_msg.response.confidence = 0.5
        request_msg.response.label = 'UNSURE'
        request_msg.image = goal_handle.request.image
        self.request_pub.publish(request_msg)

        # Initialize action feedback
        action_feedback = ImageQuery.Feedback()
        action_feedback.header = goal_handle.request.header
        action_feedback.params.detector_id = det.id
        action_feedback.params.query = det.query
        action_feedback.params.name = det.name
        action_feedback.params.patience_time = patience_time
        action_feedback.params.confidence_threshold = confidence_threshold
        action_feedback.params.human_review = human_review
        action_feedback.response.image_query_id = iq.id
        action_feedback.response.confidence = 0.5
        action_feedback.response.label = 'UNSURE'

        # Initialize feedback message
        feedback_msg = ImageQueryFeedback()
        feedback_msg.header = goal_handle.request.header
        feedback_msg.params.detector_id = det.id
        feedback_msg.params.query = det.query
        feedback_msg.params.name = det.name
        feedback_msg.params.patience_time = patience_time
        feedback_msg.params.confidence_threshold = confidence_threshold
        feedback_msg.params.human_review = human_review
        feedback_msg.response.image_query_id = iq.id
        feedback_msg.response.confidence = 0.5
        feedback_msg.response.label = 'UNSURE'

        # Poll image query until answer is final
        time_waited_sec = 0.0
        while True:
            # Get the image query
            iq = gl.get_image_query(iq.id)
            confidence = 1.0 if iq.result.confidence is None else iq.result.confidence
            label = iq.result.label.value

            # Publish feedback
            self.get_logger().info(f'Feedback: confidence={confidence} label={label}')
            action_feedback.response.confidence = confidence
            action_feedback.response.label = label
            goal_handle.publish_feedback(action_feedback)

            feedback_msg.response.confidence = confidence
            feedback_msg.response.label = label
            self.feedback_pub.publish(feedback_msg)

            # Check if answer is final
            answer_is_final = confidence is None or \
                              confidence >= confidence_threshold or \
                              time_waited_sec > patience_time
            
            if answer_is_final:
                goal_handle.succeed()
                result = ImageQuery.Result()
                result.header = goal_handle.request.header
                result.params.detector_id = det.id
                result.params.query = det.query
                result.params.name = det.name
                result.params.patience_time = patience_time
                result.params.confidence_threshold = confidence_threshold
                result.params.human_review = human_review
                result.response.image_query_id = iq.id
                result.response.confidence = confidence
                result.response.label = label
                result.image = goal_handle.request.image
                self.get_logger().info(f'Result: confidence={confidence} label={label}')
                break
            
            time.sleep(POLLING_PERIOD_SEC)
            time_waited_sec += POLLING_PERIOD_SEC
        
        # Publish result message
        result_msg = ImageQueryResult()
        result_msg.header = goal_handle.request.header
        result_msg.params.detector_id = det.id
        result_msg.params.query = det.query
        result_msg.params.name = det.name
        result_msg.params.patience_time = patience_time
        result_msg.params.confidence_threshold = confidence_threshold
        result_msg.params.human_review = human_review
        result_msg.response.image_query_id = iq.id
        result_msg.response.confidence = confidence
        result_msg.response.label = label
        result_msg.image = goal_handle.request.image
        self.result_pub.publish(result_msg)

        return result

def main(args=None):
    rclpy.init(args=args)
    iq_action_server = IQActionServer()
    rclpy.spin(iq_action_server)

if __name__ == '__main__':
    main()