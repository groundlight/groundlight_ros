import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from gl_interfaces.srv import ImageQuery

from groundlight import Groundlight

bridge = CvBridge()

gl = Groundlight() # Connect to Groundlight

class ImageQueryServer(Node):

    def __init__(self):
        super().__init__('groundlight_service')
        self.srv = self.create_service(ImageQuery, 'image_query', self.submit_image_query)

        self.get_logger().info(f'Groundlight image query server is online!')

    def submit_image_query(self, request, response):
        self.get_logger().info(f'Incoming request for detector {request.detector_id}!')

        cv_image = bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')

        image_query = gl.submit_image_query(detector=request.detector_id, image=cv_image)

        confidence = image_query.result.confidence
        if confidence is None:
            confidence = 1.0
            
        response.id = image_query.id
        response.query = image_query.query
        response.label = image_query.result.label.value
        response.confidence = confidence

        self.get_logger().info(f'Answer received from Groundlight: {response}')

        return response

def main():
    rclpy.init()

    image_query_server = ImageQueryServer()

    while rclpy.ok():
        rclpy.spin(image_query_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()