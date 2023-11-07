import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Point, PoseStamped
# from visualization_msgs.msg import Marker
# from builtin_interfaces.msg import Duration

class EndEffectorPoseNode(Node):
    def __init__(self):
        super().__init__('end_effector_pose_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'end_effector_pose', 10)
        self.get_logger().info('End Effector Pose Node has been started.')

    def get_end_effector_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'flange', rclpy.time.Time())
            # self.publish_marker(transform)
            self.publish_pose(transform)
            return transform
        except Exception as e:
            self.get_logger().error('Could not get end effector pose: %s' % str(e))
            return None

    # def publish_marker(self, transform: TransformStamped):
    #     marker = Marker()
    #     marker.header.frame_id = transform.header.frame_id
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.type = Marker.ARROW
    #     marker.action = Marker.ADD

    #     # Convert translation to Point
    #     marker.pose.position = Point(
    #         x=transform.transform.translation.x,
    #         y=transform.transform.translation.y,
    #         z=transform.transform.translation.z,
    #     )

    #     marker.pose.orientation = transform.transform.rotation

    #     # Scale is size of the arrow
    #     marker.scale.x = 0.2
    #     marker.scale.y = 0.03
    #     marker.scale.z = 0.03

    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0

    #     # Lifetime of the marker
    #     marker.lifetime = Duration(sec=0)  # 0 means never to auto-delete

    #     # Publish the marker
    #     self.marker_publisher.publish(marker)

    def publish_pose(self, transform: TransformStamped):
        pose_stamped = PoseStamped()
        pose_stamped.header = transform.header
        pose_stamped.pose.position = Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            z=transform.transform.translation.z
        )
        pose_stamped.pose.orientation = transform.transform.rotation
        # Publish the PoseStamped message
        self.pose_publisher.publish(pose_stamped)

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
