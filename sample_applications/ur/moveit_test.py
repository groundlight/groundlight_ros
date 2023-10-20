import sys
import rclpy
from rclpy.node import Node
from moveit2 import MoveItCommander
from moveit2 import roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose

class MoveToolNode(Node):
    def __init__(self):
        super().__init__('move_tool_node')
        
        # Initialize MoveIt
        roscpp_initialize(sys.argv)
        moveit_commander = MoveItCommander()
        
        # Get the RobotCommander and PlanningSceneInterface
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        
        # Get the manipulator group
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)
        
        # Get the current pose of the end effector
        current_pose = group.get_current_pose().pose
        
        # Modify the z-component of the position (move upwards)
        move_distance = 0.1  # 10cm
        current_pose.position.z += move_distance
        
        # Plan and execute
        group.set_pose_target(current_pose)
        plan = group.plan()
        group.execute(plan, wait=True)
        
        # Clean up
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        

def main(args=None):
    rclpy.init(args=args)
    move_tool_node = MoveToolNode()
    rclpy.spin(move_tool_node)
    move_tool_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
