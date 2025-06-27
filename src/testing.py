import time
import math
from humanoid_sdk import DynamixelRobot
import numpy as np

# Constants
DEG_TO_RAD = math.pi / 180
MAX_RAD = 40 * DEG_TO_RAD  # ~0.698 radians

# Initialize Robot (Simulation Only)
robot = DynamixelRobot(simulation_only=True)

# Optional: Show robot joint info
robot.debug_joint_info()

# # ------------------- FORWARD KINEMATICS TEST -------------------

# print("\n=== Forward Kinematics Test ===")
# time.sleep(0)
# # All joint angles within ±40 degrees in radians
# right_radians = [0, 0, 0, 0, 0, 0, 0]
# fk_result = robot.perform_fk_left_arm(right_radians, input_format='radians', update_simulation=True)
# print("Right Arm FK Result:", fk_result)
# time.sleep(3)
# robot.debug_joint_info()
# time.sleep(3)

print("\n=== Orientation Conversion Test ===")

# Euler to Quaternion Conversion Test
# quat = robot.rpy_to_quaternion(90, 0, 90)
# robot.move_right_hand_cartesian(0.008047678000000003, -0.3032837602360544, -0.5864664231233935, orientation=quat)
robot.move_right_hand_cartesian(0.2, 0, 0.2, euler=[0, 90, 90])
# robot.debug_joint_info()
# time.sleep(5)
# quat = robot.rpy_to_quaternion(-90, 0, 90)
# robot.move_left_hand_cartesian(0.008047678000000003, 0.3032837602360544, -0.5864664231233935, orientation=quat)
robot.move_left_hand_cartesian(0.2, 0, 0.2, euler=[0, 90, 90])
robot.debug_joint_info()
time.sleep(5)
# # ------------------- ROUND-TRIP TEST -------------------

# print("\n=== FK -> IK Round Trip Test ===")

# # Joint angles under ±40 degrees
# test_joints = [0.1, -0.15, 0.2, -0.25, 0.3, 0.1, -0.1]
# fk_result = robot.perform_fk_right_arm(test_joints, input_format='radians', update_simulation=True)

# target_pos = fk_result['position']
# target_ori = fk_result['orientation']

# robot.move_right_hand_cartesian(*target_pos, orientation=target_ori)
# final_fk = robot.get_current_fk()
# final_pos = final_fk['right_arm']['position']

# pos_error = np.linalg.norm(np.array(target_pos) - np.array(final_pos))
# print(f"Round-trip position error: {pos_error:.6f} meters")

# # ------------------- END -------------------

# print("\n=== Testing Complete ===")
