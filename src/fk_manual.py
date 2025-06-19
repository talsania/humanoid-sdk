# import math
# from ikpy.chain import Chain

# right_urdf = "/Users/dhartipatel/Downloads/actuator_sdk/urdf/right_arm_URDF.urdf"

# # Build right arm chain from j11 to j17
# right_full = Chain.from_urdf_file(right_urdf)
# r_names = [l.name for l in right_full.links]
# r_start, r_end = r_names.index('j11'), r_names.index('j17')

# right_links = right_full.links[r_start:r_end + 2]  # include end-effector
# active_r = [True] * 7 + [False]  # 7 joints, 1 end

# right_chain = Chain(name="right_arm", links=right_links, active_links_mask=active_r)

# # Input Dynamixel ticks and convert to radians
# tick_values = [2048, 2048, 2048, 2048, 2048, 2048, 2048]
# radians = [(tick - 2048) * (math.pi / 2048) for tick in tick_values]

# # 🔥 Pad to match number of links (8)
# radians += [0.0]  # add dummy value for the non-actuated end-effector link

# # Run FK
# frame = right_chain.forward_kinematics(radians)
# pos = frame[:3, 3]

# print("\n📍 End-effector position from j11–j17:")
# print(f"X: {pos[0]:.3f}, Y: {pos[1]:.3f}, Z: {pos[2]:.3f}")


import math
from ikpy.chain import Chain

# Path to your left arm URDF
left_urdf = "/Users/dhartipatel/Downloads/actuator_sdk/urdf/left_arm_URDF.urdf"

# Build left arm chain from j21 to j27
left_full = Chain.from_urdf_file(left_urdf)
l_names = [l.name for l in left_full.links]
l_start, l_end = l_names.index('j21'), l_names.index('j27')

# Extract relevant links
left_links = left_full.links[l_start:l_end + 2]  # include end-effector
active_l = [True] * 7 + [False]  # 7 joints, last is end-effector (fixed)

# Create partial chain
left_chain = Chain(name="left_arm", links=left_links, active_links_mask=active_l)

# Input Dynamixel tick values for j21 to j27
tick_values = [2048, 2048, 2048, 2048, 2048, 2048, 2048]

# Convert to radians
radians = [(tick - 2048) * (math.pi / 2048) for tick in tick_values]

# Pad with dummy for end-effector (non-joint link)
radians += [0.0]

# Forward kinematics
frame = left_chain.forward_kinematics(radians)
pos = frame[:3, 3]

print("\n📍 End-effector position from j21–j27:")
print(f"X: {pos[0]:.3f}, Y: {pos[1]:.3f}, Z: {pos[2]:.3f}")
