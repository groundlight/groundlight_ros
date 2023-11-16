import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped, Point

from gl_interfaces.msg import ImageQueryData

from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

class EndEffectorPoseNode(Node):
    def __init__(self):
        super().__init__('groundlight_markers')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.declare_parameter('camera_link', 'camera_link')
        self.camera_link = self.get_parameter('camera_link').get_parameter_value().string_value

        # Feedback subscription
        self.iq_feedback_subscription = self.create_subscription(
            ImageQueryData,
            '/groundlight/feedback',
            self.feedback_callback,
            10
        )
        self.iq_feedback_subscription

        # Result subscription
        self.iq_result_subscription = self.create_subscription(
            ImageQueryData,
            '/groundlight/result',
            self.result_callback,
            10
        )
        self.iq_result_subscription

        self.tracked_image_queries = {}
        self.iq_id_counter = 0

        self.get_logger().info('Groundlight marker node has been started.')

    def feedback_callback(self, msg: ImageQueryData):

        # check if the iq is new
        if msg.id not in self.tracked_image_queries.keys():
            self.tracked_image_queries[msg.id] = {
                'marker_id': self.iq_id_counter,
                'pose': self.pose,
            }
            self.iq_id_counter += 1

        if msg.label == 'YES':
            color = (0.0, 1.0, 0.0, 0.2)
        elif msg.label == 'NO':
            color = (1.0, 0.0, 0.0, 0.2)
        else:
            color = (1.0, 1.0, 0.0, 0.2)

        marker_id = self.tracked_image_queries[msg.id]['marker_id']
        pose = self.tracked_image_queries[msg.id]['pose']
        self.publish_marker(marker_id, pose, color)

    def result_callback(self, msg: ImageQueryData):

        if msg.label == 'YES':
            color = (0.0, 1.0, 0.0, 1.0)
        elif msg.label == 'NO':
            color = (1.0, 0.0, 0.0, 1.0)
        else:
            color = (1.0, 1.0, 0.0, 1.0)

        marker_id = self.tracked_image_queries[msg.id]['marker_id']
        pose = self.tracked_image_queries[msg.id]['pose']
        self.publish_marker(marker_id, pose, color)

        self.get_logger().info(f'Got a result! {msg}')

    def publish_marker(self, 
                       marker_id: int, 
                       pose_stamped: PoseStamped, 
                       color: tuple) -> None:
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = marker_id

        marker.pose.position = pose_stamped.pose.position
        marker.pose.orientation = pose_stamped.pose.orientation

        marker.scale.x = 0.2
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        marker.lifetime = Duration(sec=30)  # 0 means never to auto-delete

        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPoseNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.get_end_effector_pose()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
