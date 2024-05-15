import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Duration
import math

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

class FrustumPublisher(Node):
    def __init__(self):
        super().__init__('frustum_publisher', namespace='groundlight')
        self.publisher_ = self.create_publisher(Marker, 'frustum_marker', 10)
        self.subscription = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.handle_trigger, 
            10
        )
        self.fov_horizontal = math.radians(60)  # 60 degrees horizontal FOV
        self.fov_vertical = math.radians(45)    # 45 degrees vertical FOV
        self.frustum_depth = 5.0                # 5 meters depth

        self.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.4)

        self.create_service(Trigger, 'activate_frustum', self.activate_frustum)
        self.create_service(Trigger, 'deactivate_frustum', self.deactivate_frustum)

    def activate_frustum(self, request, response):
        self.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.4)
        return response

    def deactivate_frustum(self, request, response):
        self.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.4)
        return response

    def handle_trigger(self, msg):
        # This function gets called whenever a message is received on 'trigger_topic'
        marker = Marker()
        marker.header = Header(frame_id="camera_link", stamp=self.get_clock().now().to_msg())
        marker.ns = "frustum_marker"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # Calculate points based on the FOV and the frustum depth
        hw = math.tan(self.fov_horizontal / 2) * self.frustum_depth
        hh = math.tan(self.fov_vertical / 2) * self.frustum_depth

        # Define the points for the frustum
        points = [
            Point(x=0.0, y=0.0, z=0.0), Point(x=self.frustum_depth, y=-hw, z=-hh),
            Point(x=0.0, y=0.0, z=0.0), Point(x=self.frustum_depth, y=hw, z=-hh),
            Point(x=0.0, y=0.0, z=0.0), Point(x=self.frustum_depth, y=hw, z=hh),
            Point(x=0.0, y=0.0, z=0.0), Point(x=self.frustum_depth, y=-hw, z=hh),
            Point(x=self.frustum_depth, y=-hw, z=-hh), Point(x=self.frustum_depth, y=hw, z=-hh),
            Point(x=self.frustum_depth, y=hw, z=-hh), Point(x=self.frustum_depth, y=hw, z=hh),
            Point(x=self.frustum_depth, y=hw, z=hh), Point(x=self.frustum_depth, y=-hw, z=hh),
            Point(x=self.frustum_depth, y=-hw, z=hh), Point(x=self.frustum_depth, y=-hw, z=-hh),
        ]
        
        marker.points = points

        # Line thickness
        marker.scale.x = 0.005  # Line width
        
        # Line color
        marker.color = self.color
        
        # Ensure the line persists until it is explicitly removed or until a timeout
        marker.lifetime = Duration(sec=1, nanosec=0)  # Permanent marker

        self.publisher_.publish(marker)
        

def main(args=None):
    rclpy.init(args=args)
    node = FrustumPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
