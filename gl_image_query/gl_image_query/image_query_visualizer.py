import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import PoseStamped

from gl_interfaces.msg import ImageQueryData

class ImageQueryVisualizer(Node):
    def __init__(self):
        super().__init__('image_query_visualizer')

        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Pose subscripton
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            'end_effector_pose',
            self.pose_callback,
            10
        )

        self.pose_subscription  # prevent unused variable warning
        self.pose = PoseStamped()

        # Feedback subscription
        self.iq_feedback_subscription = self.create_subscription(
            ImageQueryData,
            '/image_query/feedback',
            self.feedback_callback,
            10
        )
        self.iq_feedback_subscription

        # Result subscription
        self.iq_result_subscription = self.create_subscription(
            ImageQueryData,
            '/image_query/result',
            self.result_callback,
            10
        )
        self.iq_result_subscription

        self.tracked_image_queries = {}
        self.iq_id_counter = 0

        self.get_logger().info('Groundlight image query visualizer node started!')

    def pose_callback(self, msg: PoseStamped):
        self.pose = msg

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
    image_query_visualizer = ImageQueryVisualizer()
    rclpy.spin(image_query_visualizer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
