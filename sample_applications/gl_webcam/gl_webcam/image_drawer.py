import rclpy
from rclpy.node import Node
import cv2

from cv_bridge import CvBridge

from gl_interfaces.msg import ImageQueryRequest, ImageQueryResult
from sensor_msgs.msg import Image

FONT_SCALE = 0.75

class ImageDrawer(Node):
    def __init__(self):
        super().__init__('image_drawer', namespace='groundlight')
        self.subscription_string = self.create_subscription(
            ImageQueryRequest, 
            '/groundlight/request', 
            self.request_callback, 
            10)
        self.subscription_int = self.create_subscription(
            ImageQueryResult, 
            '/groundlight/result', 
            self.result_callback, 
            10)
        
        self.publisher = self.create_publisher(Image, '/image_with_text', 10)
        
        self.bridge = CvBridge()
        self.images = {}

    def request_callback(self, msg):
        # convert to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
        self.images[msg.response.image_query_id] = cv_image # hang on to the image so we can use it when the result comes in

        # draw text on the image
        text = msg.params.query
        cv2.putText(cv_image, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, (255, 255, 255), 2)

        # convert to ROS image
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        self.publisher.publish(image_msg)

    def result_callback(self, msg):
        cv_image = self.images[msg.response.image_query_id]

        # draw text on image
        text = f'The answer is {msg.response.label}'
        if msg.response.label == "YES":
            color = (0, 255, 0)
        elif msg.response.label == "NO":
            color = (0, 0, 255)
        else:
            color = (255, 255, 0)
        cv2.putText(cv_image, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, color, 2)

        confidence_percent = round(msg.response.confidence * 100, 2)
        text = f'Confidence: {confidence_percent}%'
        cv2.putText(cv_image, text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, (255, 255, 255), 2)

        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        self.publisher.publish(image_msg)
        
        # remove the image, we don't need it anymore
        del self.images[msg.response.image_query_id]

def main(args=None):
    rclpy.init(args=args)
    node = ImageDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
