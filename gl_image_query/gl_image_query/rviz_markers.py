import rclpy
from rclpy.node import Node

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from gl_interfaces.msg import ImageQueryRequest, ImageQueryFeedback, ImageQueryResult

from math import pi
from transformations import quaternion_about_axis, quaternion_multiply

import tkinter as tk

from sensor_msgs.msg import Image as ROSImage
from PIL import Image as PILImage
from PIL import ImageTk
import cv2
from cv_bridge import CvBridge

NON_FINAL_TRANSPARENCY = 0.3
FINAL_TRANSPARENCY = 1.0

def rotate_quaternion(quaternion, angle, axis):
    """Rotate quaternion by a specific angle around an axis."""
    rotation_quaternion = quaternion_about_axis(angle, axis)
    return quaternion_multiply(rotation_quaternion, quaternion)

class ImageQueryRVizMarkers(Node):
    def __init__(self):
        super().__init__('groundlight_markers')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.marker_server = InteractiveMarkerServer(self, 'interactive_groundlight_markers')
        self.marker_counter = 0

        self.request_sub = self.create_subscription(
            ImageQueryRequest,
            '/groundlight/request',
            self.request_callback,
            10
        )

        self.feedback_sub = self.create_subscription(
            ImageQueryFeedback,
            '/groundlight/feedback',
            self.feedback_callback,
            10
        )

        self.result_sub = self.create_subscription(
            ImageQueryResult,
            '/groundlight/result',
            self.result_callback,
            10
        )

        self.iq_to_marker_map = {}
        self.iq_results = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the Tkinter main window and then withdraw it, we will only display popups, no main window
        self.root = tk.Tk()
        self.root.withdraw()

        self.get_logger().info('Groundlight RViz marker node has started.')

    def ros_image_to_pil(self, image_msg: ROSImage, new_width=640) -> PILImage:
        """Convert a ROS2 image message to a PIL Image.
        """
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

        if 'rgb8' in image_msg.encoding:
            pil_image = PILImage.fromarray(cv_image, 'RGB')
        elif 'bgr8' in image_msg.encoding:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(cv_image, 'RGB')
        else:
            pil_image = PILImage.fromarray(cv_image, 'L')

        # Calculate the new height to maintain the aspect ratio
        width_percent = (new_width / float(pil_image.size[0]))
        new_height = int((float(pil_image.size[1]) * float(width_percent)))

        # Resize the image using ANTIALIAS filter for high quality
        resized_image = pil_image.resize((new_width, new_height), PILImage.ANTIALIAS)

        return resized_image

    def create_interactive_marker(self, pose_stamped, marker_id, transparency, image_query_request):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = pose_stamped.header.frame_id
        int_marker.pose = pose_stamped.pose
        int_marker.name = f"marker_{marker_id}"

        # store the values from the image query request so that we can present them in the popup
        self.iq_results[int_marker.name] = {
            'iq_id': image_query_request.response.image_query_id,
            'image': self.ros_image_to_pil(image_query_request.image),
            'query': image_query_request.params.query,
        }

        print(self.iq_results)

        # Create the marker itself
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = transparency

        # Create a non-interactive control which contains the marker
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(marker)
        box_control.interaction_mode = InteractiveMarkerControl.BUTTON

        int_marker.controls.append(box_control)

        # Add the interactive marker to our collection and to the server
        self.marker_server.insert(int_marker, feedback_callback=self.on_click)

        self.marker_server.applyChanges()
        return int_marker

    def on_click(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.get_logger().info(f'Marker {feedback.marker_name} clicked at position ({feedback.pose.position.x}, {feedback.pose.position.y}, {feedback.pose.position.z})')

            iq_id = self.iq_results[feedback.marker_name]['iq_id']
            query = self.iq_results[feedback.marker_name]['query']
            image = self.iq_results[feedback.marker_name]['image']
            self.launch_popup(iq_id, query, image)

    def launch_popup(self, iq_id: str, query: str, image) -> None:

        # Create a new top-level window
        popup = tk.Toplevel()
        popup.title("Image Query Information")

        text = f'Image query ID: {iq_id}'
        iq_id_label = tk.Label(popup, text=text)
        iq_id_label.pack()

        query_label = tk.Label(popup, text=query)
        query_label.pack()

        tk_image = ImageTk.PhotoImage(image)
        label = tk.Label(popup, image=tk_image)
        label.image = tk_image 
        label.pack()


    def request_callback(self, msg: ImageQueryRequest):
        pose_stamped = self.get_pose(msg.header.frame_id, msg.header.stamp)
        if pose_stamped is None:
            self.get_logger().error('Could not determine pose for image query.')
            return

        int_marker = self.create_interactive_marker(pose_stamped=pose_stamped, 
                                                    marker_id=self.marker_counter, 
                                                    transparency=NON_FINAL_TRANSPARENCY,
                                                    image_query_request=msg)
        self.iq_to_marker_map[msg.response.image_query_id] = int_marker

        self.marker_counter += 1

    def feedback_callback(self, msg: ImageQueryFeedback):
        iq_id = msg.response.image_query_id
        int_marker = self.iq_to_marker_map.get(iq_id)
        if int_marker is None:
            return # No marker found
        

        # Update the marker color based on feedback
        color = (1.0, 0.0, 0.0, NON_FINAL_TRANSPARENCY)  # Default to red
        if msg.response.label == 'YES':
            color = (0.0, 1.0, 0.0, NON_FINAL_TRANSPARENCY)
        elif msg.response.label == 'NO':
            color = (1.0, 0.0, 0.0, NON_FINAL_TRANSPARENCY)

        for control in int_marker.controls:
            for marker in control.markers:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        
        self.marker_server.insert(int_marker, feedback_callback=self.on_click)
        self.marker_server.applyChanges()

    def result_callback(self, msg: ImageQueryResult):
        iq_id = msg.response.image_query_id
        int_marker = self.iq_to_marker_map.get(iq_id)
        if int_marker is None:
            return # No marker found

        # Final update to marker color
        color = (1.0, 1.0, 0.0, FINAL_TRANSPARENCY)  # Default to yellow
        if msg.response.label == 'YES':
            color = (0.0, 1.0, 0.0, FINAL_TRANSPARENCY)
        elif msg.response.label == 'NO':
            color = (1.0, 0.0, 0.0, FINAL_TRANSPARENCY)

        for control in int_marker.controls:
            for marker in control.markers:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        self.marker_server.insert(int_marker, feedback_callback=self.on_click)
        self.marker_server.applyChanges()

    def get_pose(self, source_frame: str, time_stamp=None, target_frame='map'):
        if time_stamp is None:
            time_stamp = self.get_clock().now().to_msg()

        # Wait for up to 1 second for the transformation to become available
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, time_stamp,
                timeout=rclpy.duration.Duration(seconds=1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Failed to transform from {source_frame} to {target_frame}: {str(e)}')
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header = transform.header
        pose_stamped.pose.position.x = transform.transform.translation.x
        pose_stamped.pose.position.y = transform.transform.translation.y
        pose_stamped.pose.position.z = transform.transform.translation.z

        # the pose seems to be off by 90 degrees for some reason, as a workaround, we'll just rotate it here
        original_quat = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w] 
        angle = - pi / 2  
        axis = [0, 1, 0]  # Z-axis
        rotated_quat = rotate_quaternion(original_quat, angle, axis)

        pose_stamped.pose.orientation.x = rotated_quat[0]
        pose_stamped.pose.orientation.y = rotated_quat[1]
        pose_stamped.pose.orientation.z = rotated_quat[2]
        pose_stamped.pose.orientation.w = rotated_quat[3]

        return pose_stamped

def main(args=None):
    rclpy.init(args=args)
    node = ImageQueryRVizMarkers()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        node.root.update_idletasks()
        node.root.update()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
