import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge

from gl_interfaces.action import ImageQuery
from gl_interfaces.msg import ImageQueryRequest, ImageQueryFeedback, ImageQueryResult, ImageQueryParams, ImageQueryResponse

from groundlight import Groundlight

from typing import Union
import time

bridge = CvBridge()

gl = Groundlight()

POLLING_PERIOD_SEC = .1

class IQActionServer(Node):
    def __init__(self):
        """
        A ROS 2 Action Server that wraps the Groundlight image query functionality.

        The Action Server is called by an Action Client. The client can either wait for the server to return a result or move on immediately.

        Incoming requests, feedback and results are republished on separate topics so that other nodes can subscribe.
        """
        super().__init__('action_server', namespace='groundlight')
        
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

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
        wait = 30.0 if goal_handle.request.params.wait == 0.0 else goal_handle.request.params.wait
        confidence_threshold = det.confidence_threshold if goal_handle.request.params.confidence_threshold == 0.0 else goal_handle.request.params.confidence_threshold
        human_review = 'DEFAULT' if goal_handle.request.params.human_review == '' else goal_handle.request.params.human_review

        # Submit image query
        iq = gl.ask_async(det, 
                          image=cv_image, 
                          confidence_threshold=confidence_threshold,
                          human_review=human_review)
        self.get_logger().info(f'Processing image query: {iq.query}')

        params = ImageQueryParams()
        params.detector_id = det.id
        params.query = det.query
        params.name = det.name
        params.wait = wait
        params.confidence_threshold = confidence_threshold
        params.human_review = human_review

        response = ImageQueryResponse()
        response.image_query_id = iq.id
        response.confidence = 0.5
        response.label = 'UNSURE'

        # Announce that an image query has been submitted
        request_msg = ImageQueryRequest()
        request_msg.header = goal_handle.request.header
        request_msg.params = params
        request_msg.response = response
        request_msg.image = goal_handle.request.image
        self.request_pub.publish(request_msg) 

        # Initialize feedback message and action feedback
        action_feedback = ImageQuery.Feedback()
        action_feedback.header = goal_handle.request.header
        action_feedback.params = params
        action_feedback.response = response

        feedback_msg = ImageQueryFeedback()
        feedback_msg.header = goal_handle.request.header
        feedback_msg.params = params
        feedback_msg.response = response

        # Poll image query until answer is final
        t1 = time.time()
        while True:
            # Get the image query
            iq = gl.get_image_query(iq.id)
            confidence = 1.0 if iq.result.confidence is None else iq.result.confidence
            label = iq.result.label.value

            # Update the response 
            response.confidence = confidence
            response.label = label

            # Publish action feedback
            self.get_logger().debug(f'Feedback: {self.iq_msg_to_str(action_feedback)}')
            action_feedback.response = response
            goal_handle.publish_feedback(action_feedback)

            # Publish feedback message
            feedback_msg.response = response
            self.feedback_pub.publish(feedback_msg)

            # Check if answer is final
            time_waited_so_far = time.time() - t1
            answer_is_final = confidence is None or \
                              confidence >= confidence_threshold or \
                              time_waited_so_far > wait
            
            if answer_is_final:
                goal_handle.succeed()
                result = ImageQuery.Result()
                result.header = goal_handle.request.header
                result.params = params
                result.response = response
                self.get_logger().info(f'Result: {self.iq_msg_to_str(result)}')
                break
            else:
                time.sleep(POLLING_PERIOD_SEC)
        
        # Polling has concluded, time to publish the final result message
        result_msg = ImageQueryResult()
        result_msg.header = goal_handle.request.header
        result_msg.params = params
        result_msg.response = response
        result_msg.image = goal_handle.request.image
        self.result_pub.publish(result_msg)

        # Return the result to the action client
        return result
    
    def iq_msg_to_str(self, msg: Union[ImageQueryRequest, ImageQueryFeedback, ImageQueryResult]) -> str:
        return (
            f'label={msg.response.label} confidence={msg.response.confidence:.4f} '
            f'confidence_threshold={msg.params.confidence_threshold:.4f} id={msg.response.image_query_id}'
        )

def main(args=None):
    rclpy.init(args=args)
    iq_action_server = IQActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(iq_action_server)

    try:
        executor.spin()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()