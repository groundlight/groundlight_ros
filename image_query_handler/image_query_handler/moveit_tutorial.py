import sys
import rclpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class MoveGroupPythonInterfaceTutorial:
    def __init__(self):
        # Initialize moveit_commander and a ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rclpy.init()

        # Provide the name of the move group to be used
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def go_to_pose_goal(self, pose):
        move_group = self.move_group

        # Set the target pose
        move_group.set_pose_target(pose)

        # Call the planner to compute the plan and execute it
        plan = move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses
        move_group.clear_pose_targets()

    def main(self):
        # Define a series of poses
        poses = [
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(x=0.4, y=0.1, z=0.4),
                orientation=geometry_msgs.msg.Quaternion(x=0, y=0, z=0, w=1),
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(x=0.5, y=0.0, z=0.6),
                orientation=geometry_msgs.msg.Quaternion(x=0, y=0, z=0, w=1),
            )
        ]

        for pose in poses:
            self.go_to_pose_goal(pose)

        # Shutdown the ROS node
        rclpy.shutdown()

if __name__ == '__main__':
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.main()
