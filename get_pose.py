import moveit_commander
import sys

# Initialize moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

# Create RobotCommander object
robot = moveit_commander.RobotCommander()

# Get the pose of the end effector
end_effector_pose = robot.get_link("wrist_3_joint").pose()

print(end_effector_pose)