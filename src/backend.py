import time
import math
from humanoid_sdk import DynamixelRobot
import numpy as np

# Initialize Robot (Simulation Only)
robot = DynamixelRobot(simulation_only=True)

right_rad = [0.6, 0.8, 0.1, 0.0, 1.0, 0.1, 0.1]
left_rad = [0.1, 0.6, 0.0, 1.0, 0.0, 0.0, 0.0]
fk_result = robot.perform_fk_left_arm(right_rad, input_format='radians', update_simulation=True)
print("Right Arm FK Result:", fk_result)
time.sleep(3)
fk_result = robot.perform_fk_right_arm(left_rad, input_format='radians', update_simulation=True)
print("Left Arm FK Result:", fk_result)
time.sleep(3)

#gripper control

# robot.open_left_hand_gripper()
# robot.open_right_hand_gripper()
# time.sleep(2)
# robot.close_left_hand_gripper()
# robot.close_right_hand_gripper()
# time.sleep(2)

#head control


#Cartesian control
robot.move_right_hand_cartesian(0.3,-0.2, 0, euler=[0, 0, 0])
time.sleep(5)
robot.move_left_hand_cartesian(0.1, 0.2, 0.3, euler=[0, 0, 0])
time.sleep(5)

# for visualization
robot.debug_joint_info()