#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers import InteractiveMarkerServer

class SimpleInteractiveMarker(Node):
    def __init__(self):
        super().__init__('simple_interactive_marker')
        self.server = InteractiveMarkerServer(self, 'test_marker')

        # Create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "my_marker"
        int_marker.description = "Simple Clickable Marker"
        int_marker.scale = 0.5

        # Create a cube marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 2.2
        box_marker.scale.y = 2.2
        box_marker.scale.z = 2.2
        box_marker.color.r = 0.5
        box_marker.color.g = 0.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0

        # Create a control for interaction
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(box_marker)
        control.interaction_mode = InteractiveMarkerControl.BUTTON

        int_marker.controls.append(control)

        # Add the marker to the server
        self.server.insert(int_marker, feedback_callback=self.handle_feedback)

        # Apply changes
        self.server.applyChanges()

    def handle_feedback(self, feedback):
        self.get_logger().info(f'Feedback received: {feedback.event_type}')
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.get_logger().info(f'Clicked at position ({feedback.pose.position.x}, {feedback.pose.position.y}, {feedback.pose.position.z})')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleInteractiveMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
