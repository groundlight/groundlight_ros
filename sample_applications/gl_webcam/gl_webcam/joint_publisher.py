import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger

class JointPublisher(Node):
    def __init__(self):
        super().__init__('laptop_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 1/ 20  # seconds
        self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0
        self.has_opened = False

        self.webcam_client = self.create_client(Trigger, '/groundlight/start_webcam_stream')
        self.activate_frustrum_client = self.create_client(Trigger, '/groundlight/activate_frustum')
        self.deactivate_frustrum_client = self.create_client(Trigger, '/groundlight/deactivate_frustum')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['laptop_hinge_joint']
        msg.position = [self.angle]
        self.publisher_.publish(msg)

        if self.angle < 1.7:
            self.angle += 0.025
        elif not self.has_opened:
            self.has_opened = True
            future = self.webcam_client.call_async(Trigger.Request())
            future.add_done_callback(self.handle_webcam_service_response)

    def handle_webcam_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug('Service call succeeded: %s' % response.message)
                self.activate_frustrum_client.call_async(Trigger.Request())
            else:
                self.get_logger().debug('Service call failed: %s' % response.message)
                self.deactivate_frustrum_client.call_async(Trigger.Request())
        except Exception as e:
            self.get_logger().error('Start webcam service call failed: %s' % str(e))
            self.deactivate_frustrum_client.call_async(Trigger.Request())

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
