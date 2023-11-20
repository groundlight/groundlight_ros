import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import Buffer, TransformListener

from gl_interfaces.msg import ImageQueryRequest, ImageQueryFeedback, ImageQueryResult

import time

NON_FINAL_TRANSPARENCY = 0.25
FINAL_TRANSPARENCY = 1.0

class ImageQueryRVizMarkers(Node):
    def __init__(self):
        """
        Spawns RViz arrow markers for each image query that is received. Subscribes to feedback and result topics to update the markers.
        """
        super().__init__('groundlight_markers')

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
        if not msg.header.frame_id:
            # If no frame_id is specified, we can't draw a marker
            return 

        # Create a marker for this new image query
        marker = Marker()
        marker.id = self.marker_counter
        marker.header.frame_id = 'base_link' #msg.header.frame_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose = self.get_pose(msg.header.frame_id).pose

        marker.scale.x = 0.2
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.lifetime = Duration(sec=0)  # 0 means never delete

        self.marker_counter += 1

        iq_id = msg.response.image_query_id
        self.iq_markers[iq_id] = marker

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
        # marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        self.marker_publisher.publish(marker)

    def get_pose(self, source_frame: str, target_frame: str = 'base_link') -> PoseStamped:
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())

        pose_stamped = PoseStamped()
        pose_stamped.header = transform.header
        pose_stamped.pose.position = Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            z=transform.transform.translation.z
        )
        pose_stamped.pose.orientation = transform.transform.rotation

        return pose_stamped

def main(args=None):
    rclpy.init(args=args)
    node = ImageQueryRVizMarkers()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
