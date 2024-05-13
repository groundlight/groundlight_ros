import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('web_cam', namespace='groundlight')
        self.publisher = self.create_publisher(Image, 'webcam_frames', 10)
        
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        self.has_started = False

        self.get_logger().info('Launching webcam publisher...')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(image_message)
        else:
            self.get_logger().warn('Frame capture failed')
            return
        
        if not self.has_started:
            self.has_started = True
            self.get_logger().info(f'Publishing webcam frames at {self.publisher.topic_name}')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = WebcamPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
