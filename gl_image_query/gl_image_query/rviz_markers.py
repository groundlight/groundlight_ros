import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from gl_interfaces.msg import ImageQueryRequest, ImageQueryFeedback, ImageQueryResult

from math import pi
from transformations import quaternion_about_axis, quaternion_multiply

NON_FINAL_TRANSPARENCY = 0.3
FINAL_TRANSPARENCY = 1.0

def rotate_quaternion(quaternion, angle, axis):
    """Rotate quaternion by a specific angle around an axis."""
    rotation_quaternion = quaternion_about_axis(angle, axis)
    return quaternion_multiply(rotation_quaternion, quaternion)

class ImageQueryRVizMarkers(Node):
    def __init__(self):
        """
        Spawns RViz arrow markers for each image query that is received. Subscribes to feedback and result topics to update the markers.
        """
        super().__init__('groundlight_markers')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.marker_publisher = self.create_publisher(Marker, '/groundlight/markers', 10)

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

        self.iq_markers = {}
        self.marker_counter = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Delete any existing markers in RViz to start with a blank slate
        marker = Marker()
        marker.header.frame_id = 'base_link' # This seems to be required for the DELETEALL action to work
        marker.action = Marker.DELETEALL
        self.marker_publisher.publish(marker)

        self.get_logger().info('Groundlight RViz marker node has started.')

    def request_callback(self, msg: ImageQueryRequest):
        # Attempt to get the pose at the time the image was captured
        pose_stamped = self.get_pose(msg.header.frame_id, msg.header.stamp)
        if pose_stamped is None:
            self.get_logger().error('Could not determine pose for image query.')
            return

        marker = Marker()
        marker.id = self.marker_counter
        marker.header.frame_id = 'map'
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose  # Use the transformed pose

        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = NON_FINAL_TRANSPARENCY
        marker.lifetime = Duration(sec=0)

        self.marker_counter += 1
        self.marker_publisher.publish(marker)
        self.iq_markers[msg.response.image_query_id] = marker

    def feedback_callback(self, msg: ImageQueryFeedback):
        iq_id = msg.response.image_query_id
        marker = self.iq_markers.get(iq_id)
        if marker is None:
            return # Got a callback for a marker we aren't tracking

        # Determine marker color
        if msg.response.label == 'YES':
            color = (0.0, 1.0, 0.0, NON_FINAL_TRANSPARENCY)
        elif msg.response.label == 'NO':
            color = (1.0, 0.0, 0.0, NON_FINAL_TRANSPARENCY)
        else:
            color = (1.0, 1.0, 0.0, NON_FINAL_TRANSPARENCY)

        self.publish_marker(marker, color)

    def result_callback(self, msg: ImageQueryResult):
        iq_id = msg.response.image_query_id
        marker = self.iq_markers.get(iq_id)
        if marker is None:
            return # Got a callback for a marker we aren't tracking

        # Determine marker color
        if msg.response.confidence < msg.params.confidence_threshold:
            color = (1.0, 1.0, 0.0, FINAL_TRANSPARENCY)
        elif msg.response.label == 'YES':
            color = (0.0, 1.0, 0.0, FINAL_TRANSPARENCY)
        elif msg.response.label == 'NO':
            color = (1.0, 0.0, 0.0, FINAL_TRANSPARENCY)
        else:
            color = (1.0, 1.0, 0.0, FINAL_TRANSPARENCY)

        self.publish_marker(marker, color)

    def publish_marker(self, marker: Marker, color: tuple) -> None:
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        self.marker_publisher.publish(marker)

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
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
