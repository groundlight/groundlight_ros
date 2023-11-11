import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ALLOWABLE_JOINT_ERROR = .001

class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('simple_inspection')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.target_joints = [
            -1.2539857069598597,
            -2.1392386595355433,
            -1.7889416853534144,
            1.5895161628723145,
            -1.447315518056051,
            0.43509066104888916,
        ]

        self.in_position = False


    def listener_callback(self, msg):
        # self.get_logger().info('Received Joint States:')
        # self.get_logger().info('Names: %s' % msg.name)
        # self.get_logger().info('Positions: %s' % msg.position)
        # self.get_logger().info('Velocities: %s' % msg.velocity)
        # self.get_logger().info('Efforts: %s' % msg.effort)

        for idx, position in enumerate(msg.position):
            if abs(position - self.target_joints[idx]) > ALLOWABLE_JOINT_ERROR:
                self.in_position = False
                break
        else:
            self.in_position = True

def main(args=None):
    rclpy.init(args=args)

    joint_state_subscriber = JointStateSubscriber()

    rclpy.spin(joint_state_subscriber)

    # Destroy the node explicitly
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
