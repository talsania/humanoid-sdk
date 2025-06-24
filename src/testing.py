import time
import math
from humanoid_sdk import DynamixelRobot
import numpy as np

# Initialize Robot (Set simulation_only=False for hardware testing)
robot = DynamixelRobot(simulation_only=True)

# Optional: Show robot joint info
robot.debug_joint_info()

# ------------------- FK TESTING -------------------

print("\n=== Forward Kinematics Tests ===")

# Test 1: FK with tick values (right arm)
right_ticks = [2048, 1800, 2200, 1900, 2100, 2000, 2048]
fk_result = robot.perform_fk_right_arm(right_ticks, update_simulation=True)
print("Right FK result:", fk_result)
time.sleep(3)

# Test 2: FK with radians (left arm)
left_radians = [-0.2, 0.3, -0.5, 0.2, -0.4, -0.1, -0.1]
fk_result = robot.perform_fk_left_arm(left_radians, input_format='radians', update_simulation=True)
print("Left FK result:", fk_result)
time.sleep(3)

# Test 3: FK for both arms
both_joints = right_ticks + [2500, 2300, 1600, 2400, 1700, 2100, 1800]
fk_result = robot.perform_fk_both_arms(both_joints, update_simulation=True)
print("Both Arms FK:", fk_result)
time.sleep(3)

# ------------------- IK TESTING -------------------

print("\n=== Inverse Kinematics Tests ===")

# Test 1: Position-only IK (Right hand)
robot.move_right_hand_cartesian(0.3, -0.2, -0.4)
time.sleep(3)

# Test 2: IK with Euler angles
robot.move_right_hand_cartesian(0.2, -0.3, -0.5, euler=(0.5, 0.2, 1.0))
time.sleep(3)

# Test 3: IK with Quaternion
robot.move_right_hand_cartesian(0.25, -0.25, -0.45, orientation=(0.7071, 0, 0, 0.7071))
time.sleep(3)

# ------------------- CONVERSION TESTING -------------------

print("\n=== Orientation Conversion Tests ===")

# Euler to Quaternion
quat = robot.euler_to_quaternion(0.5, 0.3, 0.8)
print("Euler to Quaternion:", quat)
robot.move_right_hand_cartesian(0.2, -0.2, -0.4, orientation=quat)
time.sleep(3)

# Axis-Angle to Quaternion
quat_axis = robot.axis_angle_to_quaternion([0, 0, 1], math.pi / 2)
robot.move_right_hand_cartesian(0.3, -0.2, -0.4, orientation=quat_axis)
time.sleep(3)

# Rotation Matrix to Quaternion
rot_matrix = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
quat_matrix = robot.rotation_matrix_to_quaternion(rot_matrix)
robot.move_right_hand_cartesian(0.25, -0.3, -0.4, orientation=quat_matrix)
time.sleep(3)

# ------------------- ROUND-TRIP TEST -------------------

print("\n=== FK -> IK Round Trip Test ===")
test_joints = [1800, 1700, 2300, 1900, 2200, 2000, 2100]
fk_result = robot.perform_fk_right_arm(test_joints, update_simulation=True)

target_pos = fk_result['position']
target_ori = fk_result['orientation']

robot.move_right_hand_cartesian(*target_pos, orientation=target_ori)
final_fk = robot.get_current_fk()
final_pos = final_fk['right_arm']['position']

pos_error = np.linalg.norm(np.array(target_pos) - np.array(final_pos))
print(f"Round-trip position error: {pos_error:.6f} meters")

# ------------------- END -------------------

print("\n=== Testing Complete ===")