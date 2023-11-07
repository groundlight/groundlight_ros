import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

from gl_interfaces.srv import GrabFrame


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()

        # Standard Publisher for Image and CameraInfo
        self.pub_image = self.create_publisher(Image, 'camera/image', 10)
        self.pub_camera_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz
        
        # Initialize OpenCV video capture object
        self.cap = cv2.VideoCapture(0)  # Open the first camera device
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            rclpy.shutdown()

        # Define dummy camera info so that we can view the feed in RViz. 
        # This isn't a best practice; the camera should be properly calibrated,
        # but for our purposes, calibration is not important; we just want to take pictures.
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0] # Distortion parameters.
        self.camera_info_msg.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] # Intrinsic camera matrix.
        self.camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] # Rotation matrix.
        self.camera_info_msg.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0] # Projection matrix.
        _, frame = self.cap.read()
        self.camera_info_msg.width = frame.shape[1]
        self.camera_info_msg.height = frame.shape[0]

        self.srv = self.create_service(GrabFrame, 'camera/grab_frame', self.grab_frame)

    def grab_frame(self, request, response):
        response.image = self._grab_and_convert_to_msg()
        return response

    def _grab_and_convert_to_msg(self) -> Image:
        ret, frame = self.cap.read()
        if ret:
            # Convert the image to ROS image message and publish it
            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                image_msg.header.frame_id = 'tool0'
                image_msg.header.stamp = self.get_clock().now().to_msg()
                return image_msg
            except Exception as e:
                self.get_logger().error(str(e))

    def publish_frame(self):
        image_msg = self._grab_and_convert_to_msg()

        # Publish the iamge
        self.pub_image.publish(image_msg)

        # Publish the dummy camera info
        self.camera_info_msg.header = image_msg.header
        self.pub_camera_info.publish(self.camera_info_msg)

    def on_shutdown(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.on_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()