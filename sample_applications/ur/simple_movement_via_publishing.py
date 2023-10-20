import os
import time
from typing import List

home = [-1.28, -2.58, 0.71, 1.46, -1.60, 6.26]
looking_down = [-0.91, -2.00, -1.80, 1.50, -1.34, 5.71]
grabbing = [-1.09, -2.60, -1.01, 1.50, -1.22, 5.82]

joint_configurations = [
    looking_down,
    grabbing,
    looking_down,
    home,
]

def move_to_joint_configuration(joint_configuration: List[float]):
    pass


while True:
    for jc in joint_configurations:
        time_sec = 4
        time_nanosec = 0
        command = (
        f'ros2 topic pub /scaled_joint_trajectory_controller/joint_trajectory '
        f'trajectory_msgs/msg/JointTrajectory "{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: \'\'}}, '
        f'joint_names: [\'shoulder_lift_joint\', \'elbow_joint\', \'wrist_1_joint\', \'wrist_2_joint\', '
        f'\'wrist_3_joint\', \'shoulder_pan_joint\'], '
        f'points: [{{positions: [{jc[0]}, {jc[1]}, {jc[2]}, {jc[3]}, {jc[4]}, {jc[5]}], '
        f'velocities: [], accelerations: [], effort: [], time_from_start: {{sec: {time_sec}, nanosec: {time_nanosec}}}}}]}}" '
        '--once'
        )

        # Printing or using the command string
        print(command)
        os.system(command)
        time.sleep(5)