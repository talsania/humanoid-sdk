# Dynamixel Humanoid Robot Controller

## IK/FK Testing Guide

1. [Initialization](#initialization)
2. [Forward Kinematics (FK) Functions](#forward-kinematics-fk-functions)
3. [Inverse Kinematics (IK) Functions](#inverse-kinematics-ik-functions)
4. [Helper Functions](#helper-functions)
5. [Simulation Control Functions](#simulation-control-functions)
6. [Testing Scenarios](#testing-scenarios)
7. [Parameter Reference](#parameter-reference)
8. [Error Handling](#error-handling)

## Initialization

### Basic Robot Initialization

```python
from actuator_sdk import DynamixelRobot

# For simulation testing
robot = DynamixelRobot(simulation_only=True)

# For hardware testing
robot = DynamixelRobot(device_name='/dev/ttyUSB0', simulation_only=False)
```

## Forward Kinematics (FK) Functions

### 1. perform_fk_right_arm()

**Purpose:** Compute end-effector pose for right arm given joint values

**Syntax:**
```python
result = robot.perform_fk_right_arm(joint_values, input_format='ticks', update_simulation=False)
```

**Parameters:**
- `joint_values`: List of 7 joint values for joints j11-j17
- `input_format`: 'ticks' (0-4095) or 'radians'
- `update_simulation`: True to show movement in simulation

**Returns:**
```python
{
    'position': [x, y, z],      # End-effector position in meters
    'orientation': [w, x, y, z] # Quaternion orientation
}
```

**Examples:**
```python
# Basic FK with tick values
result = robot.perform_fk_right_arm([2048, 1800, 2200, 1900, 2100, 2000, 2048])

# FK with visualization
result = robot.perform_fk_right_arm([1500, 1500, 2500, 1500, 2500, 1500, 2500], 
                                   update_simulation=True)

# FK with radian input
joint_radians = [0.2, -0.3, 0.5, -0.2, 0.4, 0.1, -0.1]
result = robot.perform_fk_right_arm(joint_radians, input_format='radians')
```

### 2. perform_fk_left_arm()

**Purpose:** Compute end-effector pose for left arm given joint values

**Syntax:**
```python
result = robot.perform_fk_left_arm(joint_values, input_format='ticks', update_simulation=False)
```

**Parameters:**
- `joint_values`: List of 7 joint values for joints j21-j27
- `input_format`: 'ticks' (0-4095) or 'radians'
- `update_simulation`: True to show movement in simulation

**Examples:**
```python
# Basic left arm FK
result = robot.perform_fk_left_arm([2500, 2300, 1600, 2400, 1700, 2100, 1800])

# Left arm FK with visualization
result = robot.perform_fk_left_arm([2200, 2400, 1600, 2100, 1800, 2000, 1900],
                                  update_simulation=True)
```

### 3. perform_fk_both_arms()

**Purpose:** Compute end-effector poses for both arms simultaneously

**Syntax:**
```python
result = robot.perform_fk_both_arms(joint_values, input_format='ticks', update_simulation=False)
```

**Parameters:**
- `joint_values`: List of 14 joint values [right_7_joints, left_7_joints]
- `input_format`: 'ticks' (0-4095) or 'radians'
- `update_simulation`: True to show movement in simulation

**Returns:**
```python
{
    'right_arm': {'position': [x, y, z], 'orientation': [w, x, y, z]},
    'left_arm': {'position': [x, y, z], 'orientation': [w, x, y, z]}
}
```

**Examples:**
```python
# Both arms FK
both_joints = [1800, 1600, 2400, 1900, 2200, 2000, 2100,  # Right arm (j11-j17)
               2200, 2400, 1600, 2100, 1800, 2000, 1900]  # Left arm (j21-j27)
result = robot.perform_fk_both_arms(both_joints, update_simulation=True)
```

### 4. get_current_fk()

**Purpose:** Get FK for current robot joint positions

**Syntax:**
```python
result = robot.get_current_fk()
```

**Returns:**
```python
{
    'right_arm': {'position': [x, y, z], 'orientation': [w, x, y, z]},
    'left_arm': {'position': [x, y, z], 'orientation': [w, x, y, z]}
}
```

**Examples:**
```python
# Get current state
current_poses = robot.get_current_fk()
print("Current right hand:", current_poses['right_arm']['position'])
print("Current left hand:", current_poses['left_arm']['position'])
```

## Inverse Kinematics (IK) Functions

### 1. move_right_hand_cartesian()

**Purpose:** Move right hand to target position with optional orientation control

**Syntax:**
```python
robot.move_right_hand_cartesian(x, y, z, relative=False, orientation=None, euler=None)
```

**Parameters:**
- `x, y, z`: Target position coordinates (meters)
- `relative`: If True, position is relative to home pose
- `orientation`: Quaternion as (w, x, y, z) tuple
- `euler`: Euler angles as (roll, pitch, yaw) tuple (radians)

**Examples:**
```python
# Basic position-only IK
robot.move_right_hand_cartesian(0.3, -0.2, -0.4)

# IK with Euler angles (roll, pitch, yaw in radians)
robot.move_right_hand_cartesian(0.2, -0.3, -0.5, euler=(0.5, 0.2, 1.0))

# IK with direct quaternion (w, x, y, z)
robot.move_right_hand_cartesian(0.25, -0.25, -0.45, orientation=(0.7071, 0, 0, 0.7071))

# Relative positioning (if home_se3 is defined)
robot.move_right_hand_cartesian(0.05, 0.05, 0.1, relative=True)
```

### 2. move_left_hand_cartesian()

**Purpose:** Move left hand to target position with optional orientation control

**Syntax:**
```python
robot.move_left_hand_cartesian(x, y, z, relative=False, orientation=None, euler=None)
```

**Parameters:** Same as move_right_hand_cartesian()

**Examples:**
```python
# Basic left arm IK
robot.move_left_hand_cartesian(0.3, 0.2, -0.4)

# Left arm with Euler orientation
robot.move_left_hand_cartesian(0.2, 0.3, -0.5, euler=(0.3, -0.4, 0.8))

# Left arm with quaternion
robot.move_left_hand_cartesian(0.25, 0.25, -0.45, orientation=(0.866, 0.5, 0, 0))
```

## Helper Functions

### 1. euler_to_quaternion()

**Purpose:** Convert Euler angles to quaternion

**Syntax:**
```python
quat = robot.euler_to_quaternion(roll, pitch, yaw)
```

**Parameters:**
- `roll`: Rotation around X-axis (radians)
- `pitch`: Rotation around Y-axis (radians)
- `yaw`: Rotation around Z-axis (radians)

**Returns:** (w, x, y, z) quaternion tuple

**Examples:**
```python
# Convert Euler to quaternion
quat = robot.euler_to_quaternion(0.5, 0.3, 0.8)
robot.move_right_hand_cartesian(0.2, -0.2, -0.4, orientation=quat)

# Different rotations
quat_roll = robot.euler_to_quaternion(1.57, 0, 0)  # 90° roll
quat_pitch = robot.euler_to_quaternion(0, 1.57, 0)  # 90° pitch
quat_yaw = robot.euler_to_quaternion(0, 0, 1.57)   # 90° yaw
```

### 2. axis_angle_to_quaternion()

**Purpose:** Convert axis-angle representation to quaternion

**Syntax:**
```python
quat = robot.axis_angle_to_quaternion(axis, angle)
```

**Parameters:**
- `axis`: 3-element list/array representing rotation axis
- `angle`: Rotation angle (radians)

**Returns:** (w, x, y, z) quaternion tuple

**Examples:**
```python
# 90° rotation around Z-axis
quat = robot.axis_angle_to_quaternion([0, 0, 1], math.pi/2)
robot.move_right_hand_cartesian(0.3, -0.2, -0.4, orientation=quat)

# 45° rotation around X-axis
quat = robot.axis_angle_to_quaternion([1, 0, 0], math.pi/4)

# 60° rotation around arbitrary axis
quat = robot.axis_angle_to_quaternion([1, 1, 0], math.pi/3)
```

### 3. rotation_matrix_to_quaternion()

**Purpose:** Convert 3x3 rotation matrix to quaternion

**Syntax:**
```python
quat = robot.rotation_matrix_to_quaternion(rotation_matrix)
```

**Parameters:**
- `rotation_matrix`: 3x3 numpy array

**Returns:** (w, x, y, z) quaternion tuple

**Examples:**
```python
import numpy as np

# 90° rotation around Z-axis matrix
rot_matrix = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
quat = robot.rotation_matrix_to_quaternion(rot_matrix)
robot.move_right_hand_cartesian(0.25, -0.3, -0.4, orientation=quat)

# Identity matrix (no rotation)
identity = np.eye(3)
quat = robot.rotation_matrix_to_quaternion(identity)
```

## Simulation Control Functions

### 1. set_simulation_pose()

**Purpose:** Directly set robot pose for visualization (fastest method)

**Syntax:**
```python
robot.set_simulation_pose(joint_values, input_format='ticks', arm='both')
```

**Parameters:**
- `joint_values`: Joint values (7 for single arm, 14 for both)
- `input_format`: 'ticks' or 'radians'
- `arm`: 'right', 'left', or 'both'

**Examples:**
```python
# Set right arm pose
robot.set_simulation_pose([1500, 1500, 2500, 1500, 2500, 1500, 2500], arm='right')

# Set both arms pose
both_arms = [1800, 1600, 2400, 1900, 2200, 2000, 2100,  # Right
             2200, 2400, 1600, 2100, 1800, 2000, 1900]  # Left
robot.set_simulation_pose(both_arms, arm='both')

# Set with radians
robot.set_simulation_pose([0.2, -0.3, 0.5, -0.2, 0.4, 0.1, -0.1],
                         input_format='radians', arm='left')
```

### 2. debug_joint_info()

**Purpose:** Display available joints and model information

**Syntax:**
```python
robot.debug_joint_info()
```

**Output:** Prints joint names, IDs, and model statistics

**Example:**
```python
# Debug joint information
robot.debug_joint_info()
```

## Testing Scenarios

### Basic FK Testing

```python
# 1. Home position FK
home_joints = [2048, 2048, 2048, 2048, 2048, 2048, 2048]
result = robot.perform_fk_right_arm(home_joints, update_simulation=True)

# 2. Extreme positions
extreme_joints = [3500, 500, 3000, 1000, 3200, 800, 2800]
result = robot.perform_fk_right_arm(extreme_joints, update_simulation=True)

# 3. Both arms different poses
both_joints = [1500, 1500, 2500, 1500, 2500, 1500, 2500,  # Right
               2500, 2500, 1500, 2500, 1500, 2500, 1500]  # Left
result = robot.perform_fk_both_arms(both_joints, update_simulation=True)
```

### Basic IK Testing

```python
# 1. Position-only IK
robot.move_right_hand_cartesian(0.3, -0.2, -0.4)

# 2. IK with orientation
robot.move_right_hand_cartesian(0.2, -0.3, -0.5, euler=(0.5, 0.2, 1.0))

# 3. Sequential movements
positions = [(0.15, -0.15, -0.3), (0.25, -0.25, -0.4), (0.35, -0.15, -0.5)]
for x, y, z in positions:
    robot.move_right_hand_cartesian(x, y, z)
    time.sleep(2)
```

### Advanced Testing Scenarios

#### Workspace Exploration

```python
# Test various reach distances and angles
distances = [0.2, 0.3, 0.4, 0.5]
angles = [0, 45, 90, 135, 180, 225, 270, 315]  # degrees

for distance in distances:
    for angle_deg in angles:
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        z = -0.3
        try:
            robot.move_right_hand_cartesian(x, y, z)
            print(f"✅ Reached ({x:.2f}, {y:.2f}, {z:.2f})")
        except Exception as e:
            print(f"❌ Failed to reach ({x:.2f}, {y:.2f}, {z:.2f})")
```

#### Orientation Sweep Testing

```python
base_pos = (0.25, -0.25, -0.4)

# Roll sweep
for angle in np.linspace(-0.5, 0.5, 10):
    robot.move_right_hand_cartesian(base_pos[0], base_pos[1], base_pos[2],
                                   euler=(angle, 0, 0))
    time.sleep(1)

# Pitch sweep
for angle in np.linspace(-0.3, 0.3, 10):
    robot.move_right_hand_cartesian(base_pos[0], base_pos[1], base_pos[2],
                                   euler=(0, angle, 0))
    time.sleep(1)

# Yaw sweep
for angle in np.linspace(-0.8, 0.8, 10):
    robot.move_right_hand_cartesian(base_pos[0], base_pos[1], base_pos[2],
                                   euler=(0, 0, angle))
    time.sleep(1)
```

#### Circular Trajectory Testing

```python
center = (0.25, -0.25, -0.4)
radius = 0.05
num_points = 12

for i in range(num_points + 1):
    angle = 2 * math.pi * i / num_points
    x = center[0] + radius * math.cos(angle)
    y = center[1] + radius * math.sin(angle)
    z = center[2]
    robot.move_right_hand_cartesian(x, y, z)
    time.sleep(1)
```

#### FK-IK Round-trip Validation

```python
# Start with known joint configuration
test_joints = [1800, 1700, 2300, 1900, 2200, 2000, 2100]

# Get FK result
fk_result = robot.perform_fk_right_arm(test_joints, update_simulation=True)
target_pos = fk_result['position']
target_ori = fk_result['orientation']

# Try to reach the same position with IK
robot.move_right_hand_cartesian(target_pos[0], target_pos[1], target_pos[2],
                               orientation=target_ori)

# Check final FK
final_fk = robot.get_current_fk()
final_pos = final_fk['right_arm']['position']

# Calculate error
pos_error = np.linalg.norm(np.array(target_pos) - np.array(final_pos))
print(f"Round-trip position error: {pos_error:.6f} meters")
```

#### Performance Testing

```python
import time

# FK speed test
test_joints = [2048, 1800, 2200, 1900, 2100, 2000, 2048]
start_time = time.time()
for _ in range(100):
    result = robot.perform_fk_right_arm(test_joints)
fk_time = time.time() - start_time
print(f"100 FK computations: {fk_time:.4f}s")

# IK speed test
start_time = time.time()
for i in range(10):
    x = 0.2 + 0.01 * i
    robot.move_right_hand_cartesian(x, -0.2, -0.4)
ik_time = time.time() - start_time
print(f"10 IK computations: {ik_time:.4f}s")
```

## Parameter Reference

### Joint ID Mapping

- **Right Arm:** j11, j12, j13, j14, j15, j16, j17 (indices 0-6)
- **Left Arm:** j21, j22, j23, j24, j25, j26, j27 (indices 7-13)
- **Head:** j31, j32
- **Grippers:** j38 (right), j48 (left)

### Coordinate System

- **X-axis:** Forward/backward (positive = forward)
- **Y-axis:** Left/right (positive = left, negative = right)
- **Z-axis:** Up/down (positive = up, negative = down)
- **Units:** Meters for position, radians for angles

### Joint Ranges (approximate)

- **Tick Range:** 0 - 4095 (center = 2048)
- **Radian Range:** -π to +π (center = 0)
- **Conversion:** `radians = (ticks - 2048) * (π / 2048)`

### Quaternion Format

- **Order:** (w, x, y, z) - scalar first
- **Normalization:** Quaternions should be normalized (|q| = 1)
- **Identity:** (1, 0, 0, 0) represents no rotation

### Euler Angle Convention

- **Order:** Roll (X), Pitch (Y), Yaw (Z)
- **Range:** Typically -π/2 to +π/2 for practical use
- **Units:** Radians

## Error Handling

### Common Errors and Solutions

#### 1. Joint Value Errors

```python
# Error: Wrong number of joints
try:
    result = robot.perform_fk_right_arm([2048, 1800, 2200])  # Only 3 joints
except ValueError as e:
    print(f"Error: {e}")  # "Right arm requires exactly 7 joint values"

# Solution: Provide exactly 7 joints
result = robot.perform_fk_right_arm([2048, 1800, 2200, 1900, 2100, 2000, 2048])
```

#### 2. Unreachable Target Errors

```python
# Error: Target outside workspace
try:
    robot.move_right_hand_cartesian(2.0, 0.0, 0.0)  # Too far
except Exception as e:
    print(f"IK failed: {e}")

# Solution: Test reachable positions first
reachable_positions = [
    (0.2, -0.2, -0.3),  # Close and safe
    (0.3, -0.3, -0.4),  # Medium reach
    (0.4, -0.4, -0.5),  # Extended reach
]
```

#### 3. Simulation Mode Errors

```python
# Error: Hardware functions in simulation
if robot.simulation_only:
    print("Running in simulation mode")
    robot.set_simulation_pose([2048]*7, arm='right')
else:
    print("Running on hardware")
    robot.write_positions(robot.right_ids, [2048]*7)
```

#### 4. Input Format Errors

```python
# Error: Wrong input format
try:
    result = robot.perform_fk_right_arm([0.1, 0.2, 0.3], input_format='ticks')
except Exception as e:
    print(f"Error: {e}")

# Solution: Match format with values
result = robot.perform_fk_right_arm([0.1, 0.2, 0.3, 0.1, 0.2, 0.0, 0.1],
                                   input_format='radians')
```

### Validation Checks

```python
# Validate joint ranges
def validate_joint_ticks(joints):
    for i, joint in enumerate(joints):
        if not (0 <= joint <= 4095):
            print(f"Warning: Joint {i} value {joint} outside valid range [0, 4095]")

# Validate position reachability
def validate_position(x, y, z):
    distance = math.sqrt(x**2 + y**2 + z**2)
    if distance > 0.8:  # Approximate max reach
        print(f"Warning: Position ({x}, {y}, {z}) may be unreachable")

# Validate quaternion
def validate_quaternion(quat):
    magnitude = math.sqrt(sum(q**2 for q in quat))
    if abs(magnitude - 1.0) > 0.01:
        print(f"Warning: Quaternion {quat} not normalized (magnitude: {magnitude})")
```

## Quick Reference Summary

### Function Categories

| Category | Functions |
|----------|-----------|
| **FK** | `perform_fk_right_arm()`, `perform_fk_left_arm()`, `perform_fk_both_arms()`, `get_current_fk()` |
| **IK** | `move_right_hand_cartesian()`, `move_left_hand_cartesian()` |
| **Conversion** | `euler_to_quaternion()`, `axis_angle_to_quaternion()`, `rotation_matrix_to_quaternion()` |
| **Simulation** | `set_simulation_pose()`, `debug_joint_info()` |

### Most Common Test Commands

```python
# FK Testing
result = robot.perform_fk_right_arm([2048, 1800, 2200, 1900, 2100, 2000, 2048], 
                                   update_simulation=True)
robot.set_simulation_pose([1500, 1500, 2500, 1500, 2500, 1500, 2500], arm='right')

# IK Testing
robot.move_right_hand_cartesian(0.3, -0.2, -0.4)
robot.move_right_hand_cartesian(0.2, -0.3, -0.5, euler=(0.5, 0.2, 1.0))
robot.move_right_hand_cartesian(0.25, -0.25, -0.45, orientation=(0.7071, 0, 0, 0.7071))

# Conversion Testing
quat = robot.euler_to_quaternion(0.5, 0.3, 0.8)
robot.move_right_hand_cartesian(0.2, -0.2, -0.4, orientation=quat)
```
