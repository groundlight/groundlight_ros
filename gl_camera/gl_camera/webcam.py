import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('web_cam', namespace='groundlight')
        self.publisher = self.create_publisher(Image, 'webcam_frames', 10)
        
        self.timer_period = 1 / 20  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.br = CvBridge()

        self.has_started = False
        self.cap = None

        self.create_service(Trigger, 'start_webcam_stream', self.start_webcam_stream)

        self.get_logger().info('Launching webcam publisher...')

    def start_webcam_stream(self, request, response):
        if not self.has_started:
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                self.has_started = True
                response.success = True
                response.message = "Camera started successfully."
                self.get_logger().info("Camera started successfully.")
            else:
                response.success = False
                response.message = "Failed to start camera."
                self.get_logger().info("Failed to start camera.")
        else:
            response.success = False
            response.message = "Camera is already running."
        return response

    def timer_callback(self):
        if self.has_started and self.cap:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.flip(frame, 1)
                image_message = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher.publish(image_message)
            else:
                self.get_logger().warn('Frame capture failed')
        else:
            # Publish a blank canvas if the webcam hasn't started
            blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            image_message = self.br.cv2_to_imgmsg(blank_frame, encoding='bgr8')
            self.publisher.publish(image_message)

    def destroy_node(self):
        if self.cap:
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
