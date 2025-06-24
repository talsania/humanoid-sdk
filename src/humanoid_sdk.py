import os
import time
import math
import numpy as np
import serial.tools.list_ports
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite
from ikpy.chain import Chain
import json
import mujoco
import mink
import mujoco.viewer
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation as R


# Control table addresses for Protocol 2.0
ADDR_TORQUE_ENABLE        = 64
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_POSITION     = 132
LEN_GOAL_POSITION         = 4
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112

ADDR_PRESENT_VELOCITY     = 128  # optional: read present velocity
ADDR_PRESENT_CURRENT      = 126  # optional: read present current
ADDR_PRESENT_TEMPERATURE  = 146  # optional: read present temperature
ADDR_PRESENT_VOLTAGE      = 144  # optional: read present voltage

# Torque control values
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0


def list_available_ports():
    """
    Returns a list of serial ports available on the system.
    Works across Windows, Linux, and macOS.
    """
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

class DynamixelRobot:
    def __init__(
        self,
        device_name: str = '',
        baud_rate: int = 4000000,
        protocol_version: float = 2.0,
        right_urdf: str = 'urdf/right_arm_URDF.urdf',
        left_urdf:  str = 'urdf/left_arm_URDF.urdf',
        simulation_only=False
    ):

        self.simulation_only = simulation_only
        # Auto-detect or verify serial port
        if not device_name:
            ports = list_available_ports()
            if not ports:
                raise IOError("No available serial ports found.")
            print("Available ports:", ports)
            device_name = ports[0]
            print(f"Auto-selecting port: {device_name}")

        if not self.simulation_only:
            self.port   = PortHandler(device_name)
            self.packet = PacketHandler(protocol_version)
            if not self.port.openPort():
                raise IOError(f"Could not open port {device_name}")
            if not self.port.setBaudRate(baud_rate):
                raise IOError(f"Could not set baud rate to {baud_rate}")


        # Joint IDs
        self.right_ids = [11,12,13,14,15,16,17]
        self.left_ids  = [21,22,23,24,25,26,27]
        self.head_ids  = [31,32]
        self.grip_r    = 38
        self.grip_l    = 48
        self.all_ids   = self.right_ids + self.left_ids + self.head_ids + [self.grip_r, self.grip_l]
        self.sequence_file = "saved_sequences.json"
        self.load_sequences()

        

        # Enable torque only if using hardware
        if not self.simulation_only:
            self.torque_on_all()

        # Predefined poses (in ticks)
        self.poses = {
            'home':   {i: 2048 for i in self.all_ids},
            'prepose': {**dict(zip(self.right_ids, [1600]*6 + [2496, 1600])),
                        **dict(zip(self.left_ids,  [1600]*7)),
                        31:2048, 32:2400,
                        self.grip_r:1500, self.grip_l:1500},
            'pose1': {i:1600 for i in self.all_ids},
            'pose2': {i:1800 for i in self.all_ids},
        }

        # Load MuJoCo models for both arms (keep for IK calculations)
        self.mj_model_r = mujoco.MjModel.from_xml_path("./urdf/right_arm.xml")
        self.mj_data_r = mujoco.MjData(self.mj_model_r)

        self.mj_model_l = mujoco.MjModel.from_xml_path("./urdf/left_arm.xml")
        self.mj_data_l = mujoco.MjData(self.mj_model_l)

        # Load full robot model for simulation display
        if self.simulation_only:
            self.mj_model_full = mujoco.MjModel.from_xml_path("./urdf/humanoid_full.xml")
            self.mj_data_full = mujoco.MjData(self.mj_model_full)
            self.viewer_full = mujoco.viewer.launch_passive(self.mj_model_full, self.mj_data_full, show_left_ui=False, show_right_ui=False)
        else:
            self.mj_model_full = None
            self.mj_data_full = None
            self.viewer_full = None


        # Configuration
        self.config_r = mink.Configuration(self.mj_model_r)
        self.config_l = mink.Configuration(self.mj_model_l)


        self.task_r = mink.FrameTask(
            frame_name="right_wrist_2",  # or whatever your end-effector body is
            frame_type="body",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        self.task_l = mink.FrameTask(
            frame_name="left_wrist_2",
            frame_type="body",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        self.posture_r = mink.PostureTask(model=self.mj_model_r, cost=1e-2)
        self.posture_l = mink.PostureTask(model=self.mj_model_l, cost=1e-2)

        self.solver = "osqp"
        self.rate = RateLimiter(frequency=40.0, warn=False)
        self.max_iters = 20
        self.pos_threshold = 0.005
        self.dt = 1.0 / 40.0

    def _sync_to_full_model(self):
        """Sync joint states from individual arm models to full robot model for display."""
        if not self.simulation_only or self.mj_data_full is None:
            return
        
        # Sync right arm joints (j11 through j17)
        right_joint_names = [f"j{11+j}" for j in range(7)]
        for i, joint_name in enumerate(right_joint_names):
            joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id >= 0 and i < len(self.config_r.q):
                self.mj_data_full.qpos[joint_id] = self.config_r.q[i]
        
        # Sync left arm joints (j21 through j27)
        left_joint_names = [f"j{21+j}" for j in range(7)]
        for i, joint_name in enumerate(left_joint_names):
            joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id >= 0 and i < len(self.config_l.q):
                self.mj_data_full.qpos[joint_id] = self.config_l.q[i]
        
        # Forward kinematics for full model and sync viewer
        mujoco.mj_forward(self.mj_model_full, self.mj_data_full)
        if self.viewer_full:
            self.viewer_full.sync()

    # ----------------------------------------------------------------------------
    # Orientation helper methods (NEW)
    # ----------------------------------------------------------------------------
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) in radians to quaternion (w, x, y, z).
        
        Args:
            roll: Rotation around X-axis (radians)
            pitch: Rotation around Y-axis (radians) 
            yaw: Rotation around Z-axis (radians)
            
        Returns:
            tuple: (w, x, y, z) quaternion
        """
        rot = R.from_euler('xyz', [roll, pitch, yaw])
        q = rot.as_quat()  # scipy returns (x, y, z, w)
        return (q[3], q[0], q[1], q[2])  # return as (w, x, y, z)
    
    @staticmethod
    def axis_angle_to_quaternion(axis, angle):
        """
        Convert axis-angle representation to quaternion (w, x, y, z).
        
        Args:
            axis: 3-element array/list representing rotation axis
            angle: Rotation angle in radians
            
        Returns:
            tuple: (w, x, y, z) quaternion
        """
        axis = np.array(axis) / np.linalg.norm(axis)  # normalize axis
        rot = R.from_rotvec(angle * axis)
        q = rot.as_quat()  # scipy returns (x, y, z, w)
        return (q[3], q[0], q[1], q[2])  # return as (w, x, y, z)
    
    @staticmethod
    def rotation_matrix_to_quaternion(rotation_matrix):
        """
        Convert 3x3 rotation matrix to quaternion (w, x, y, z).
        
        Args:
            rotation_matrix: 3x3 numpy array
            
        Returns:
            tuple: (w, x, y, z) quaternion
        """
        rot = R.from_matrix(rotation_matrix)
        q = rot.as_quat()  # scipy returns (x, y, z, w)
        return (q[3], q[0], q[1], q[2])  # return as (w, x, y, z)

    # ----------------------------------------------------------------------------
    # Torque control methods
    # ----------------------------------------------------------------------------
    def set_torque(self, dxl_ids, enable: bool = True):
        """Enable or disable torque on one or multiple actuators."""
        if isinstance(dxl_ids, int):
            dxl_ids = [dxl_ids]
        value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        for jid in dxl_ids:
            self.packet.write1ByteTxRx(self.port, jid, ADDR_TORQUE_ENABLE, value)

    def torque_on_all(self):
        """Enable torque on all configured joints."""
        self.set_torque(self.all_ids, True)

    def torque_off_all(self):
        """Disable torque on all configured joints."""
        self.set_torque(self.all_ids, False)

    def torque_on_joints(self, id_list):
        """Enable torque on a specific list of joints."""
        self.set_torque(id_list, True)

    def torque_off_joints(self, id_list):
        """Disable torque on a specific list of joints."""
        self.set_torque(id_list, False)

    # ----------------------------------------------------------------------------
    # Profile and motion settings
    # ----------------------------------------------------------------------------
    def set_profile(self, dxl_id: int, acceleration: int, velocity: int):
        self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_ACCELERATION, acceleration)
        self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_VELOCITY, velocity)

    def set_all_profiles(self, acceleration: int, velocity: int):
        for jid in self.all_ids:
            self.set_profile(jid, acceleration, velocity)
            
        # --- 1) FK helper methods ---
    def get_right_hand_pose(self):
        """Returns current end‐effector position and orientation for the right hand."""
        self.mj_data_r.qpos[:len(self.config_r.q)] = self.config_r.q
        mujoco.mj_forward(self.mj_model_r, self.mj_data_r)
        
        # Get end-effector body ID
        body_id = mujoco.mj_name2id(self.mj_model_r, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_2")
        if body_id < 0:
            raise ValueError("Body 'right_wrist_2' not found in model")
        
        # Extract position and orientation
        position = self.mj_data_r.xpos[body_id].copy()
        orientation = self.mj_data_r.xquat[body_id].copy()
        
        return {'position': position, 'orientation': orientation}

    def get_left_hand_pose(self):
        """Returns current end‐effector position and orientation for the left hand."""
        self.mj_data_l.qpos[:len(self.config_l.q)] = self.config_l.q
        mujoco.mj_forward(self.mj_model_l, self.mj_data_l)
        
        # Get end-effector body ID
        body_id = mujoco.mj_name2id(self.mj_model_l, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_2")
        if body_id < 0:
            raise ValueError("Body 'left_wrist_2' not found in model")
        
        # Extract position and orientation
        position = self.mj_data_l.xpos[body_id].copy()
        orientation = self.mj_data_l.xquat[body_id].copy()
        
        return {'position': position, 'orientation': orientation}
    
    # --- 2) Quaternion‐from‐direction utility ---
    @staticmethod
    def _quat_from_direction(direction, default_forward=np.array([0,0,1])):
        v = direction / np.linalg.norm(direction)
        f = default_forward / np.linalg.norm(default_forward)
        # handle the opposite‐vector singularity
        if np.allclose(v, -f):
            axis = np.array([1,0,0]) if abs(f[0])<0.9 else np.array([0,1,0])
            rot = R.from_rotvec(np.pi*axis)
        else:
            rot, _ = R.align_vectors([v], [f])
        q = rot.as_quat()    # scipy: (x,y,z,w)
        return (q[3], q[0], q[1], q[2])  # to (w,x,y,z)

    # ----------------------------------------------------------------------------
    # Read and write positions
    # ----------------------------------------------------------------------------
    def _tick2rad(self, tick):
        return (tick - 2048) * (math.pi / 2048.0)

    def _rad2tick(self, rad):
        return int(rad * (2048.0 / math.pi) + 2048)

    def read_position(self, dxl_id):
        pos, _, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        return pos

    def read_all_positions(self):
        """Return a dict of present positions for all actuators."""
        return {jid: self.read_position(jid) for jid in self.all_ids}

    def write_positions(self, id_list, tick_list):
        gsw = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        for jid, tk in zip(id_list, tick_list):
            data = [tk & 0xFF, (tk >> 8) & 0xFF, (tk >> 16) & 0xFF, (tk >> 24) & 0xFF]
            gsw.addParam(jid, bytearray(data))
        gsw.txPacket()
        gsw.clearParam()

    # ----------------------------------------------------------------------------
    # Direct motion commands
    # ----------------------------------------------------------------------------
    def move_right_hand_joints(self, *ticks):
        self.write_positions(self.right_ids, ticks)
    def move_left_hand_joints(self,  *ticks):
        self.write_positions(self.left_ids,  ticks)

    def open_right_hand_gripper(self, pos=2048):  self.write_positions([self.grip_r],[pos])
    def close_right_hand_gripper(self, pos=1024): self.write_positions([self.grip_r],[pos])
    def open_left_hand_gripper(self, pos=2048):   self.write_positions([self.grip_l],[pos])
    def close_left_hand_gripper(self, pos=1024):  self.write_positions([self.grip_l],[pos])

    # --- ENHANCED cartesian moves with orientation control ---
    def move_right_hand_cartesian(self, x, y, z, relative=False, orientation=None, euler=None):
        """
        Move right hand to target position with optional orientation control.
        
        Args:
            x, y, z: Target position coordinates
            relative: If True, position is relative to current home pose
            orientation: Optional quaternion as (w, x, y, z) tuple/list
            euler: Optional Euler angles as (roll, pitch, yaw) tuple/list in radians
            
        Note: If both orientation and euler are provided, orientation takes precedence.
              If neither is provided, orientation is auto-computed from motion direction.
        """
        target_pos = np.array([x, y, z])
        if relative:
            # express target in world coords: home * Δpos
            T = self.home_se3_r
            pos_world = T.apply_translation(target_pos)
        else:
            pos_world = target_pos

        # Handle orientation
        if orientation is not None:
            # orientation should be (w, x, y, z) quaternion
            if len(orientation) == 4:
                qw, qx, qy, qz = orientation
            else:
                raise ValueError("Orientation should be a 4-element quaternion (w, x, y, z)")
        elif euler is not None:
            # Convert Euler angles to quaternion
            if len(euler) == 3:
                qw, qx, qy, qz = self.euler_to_quaternion(euler[0], euler[1], euler[2])
            else:
                raise ValueError("Euler angles should be a 3-element tuple (roll, pitch, yaw) in radians")
        else:
            # auto‐compute the quaternion so +Z of tool aligns with motion dir (existing behavior)
            qw, qx, qy, qz = self._quat_from_direction(pos_world)

        T_target = mink.SE3(wxyz_xyz=[qw, qx, qy, qz,
                                      pos_world[0], pos_world[1], pos_world[2]])
        self.task_r.set_target(T_target)
        self.posture_r.set_target_from_configuration(self.config_r)

        for i in range(self.max_iters):
            vel = mink.solve_ik(self.config_r, [self.task_r, self.posture_r],
                                self.dt, self.solver, 5e-3)
            self.config_r.integrate_inplace(vel, self.dt)
            self.mj_data_r.qpos[:len(self.config_r.q)] = self.config_r.q
            mujoco.mj_forward(self.mj_model_r, self.mj_data_r)
            if self.simulation_only:
                # Update the full robot model directly for visualization
                if hasattr(self, 'mj_data_full') and self.mj_data_full is not None:
                    right_joint_ids = [11, 12, 13, 14, 15, 16, 17]
                    for i, joint_id in enumerate(right_joint_ids):
                        if i < len(self.config_r.q):
                            joint_name = f"j{joint_id}"
                            mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                            if mj_joint_id >= 0:
                                self.mj_data_full.qpos[mj_joint_id] = self.config_r.q[i]
                    
                    # Force forward kinematics and viewer update
                    mujoco.mj_forward(self.mj_model_full, self.mj_data_full)
                    if self.viewer_full:
                        self.viewer_full.sync()
                self.rate.sleep()

        if not self.simulation_only:
            q = self.config_r.q[:len(self.right_ids)]
            self.write_positions(self.right_ids,
                                 [self._rad2tick(a) for a in q])

    def move_left_hand_cartesian(self, x, y, z, relative=False, orientation=None, euler=None):
        """
        Move left hand to target position with optional orientation control.
        
        Args:
            x, y, z: Target position coordinates
            relative: If True, position is relative to current home pose
            orientation: Optional quaternion as (w, x, y, z) tuple/list
            euler: Optional Euler angles as (roll, pitch, yaw) tuple/list in radians
            
        Note: If both orientation and euler are provided, orientation takes precedence.
              If neither is provided, orientation is auto-computed from motion direction.
        """
        # Build target in either world or relative-to-home coordinates
        target_delta = np.array([x, y, z])
        if relative:
            # Express the delta in world frame by applying the home SE3
            pos_world = self.home_se3_l.apply_translation(target_delta)
        else:
            pos_world = target_delta

        # Handle orientation
        if orientation is not None:
            # orientation should be (w, x, y, z) quaternion
            if len(orientation) == 4:
                qw, qx, qy, qz = orientation
            else:
                raise ValueError("Orientation should be a 4-element quaternion (w, x, y, z)")
        elif euler is not None:
            # Convert Euler angles to quaternion
            if len(euler) == 3:
                qw, qx, qy, qz = self.euler_to_quaternion(euler[0], euler[1], euler[2])
            else:
                raise ValueError("Euler angles should be a 3-element tuple (roll, pitch, yaw) in radians")
        else:
            # Auto-compute quaternion so +Z of the tool aligns with the motion direction (existing behavior)
            qw, qx, qy, qz = self._quat_from_direction(pos_world)

        # Construct desired SE3 (w, x, y, z, px, py, pz)
        T_target = mink.SE3(wxyz_xyz=[
            qw, qx, qy, qz,
            pos_world[0], pos_world[1], pos_world[2]
        ])
        self.task_l.set_target(T_target)
        self.posture_l.set_target_from_configuration(self.config_l)

        # Run the IK loop
        for i in range(self.max_iters):
            vel = mink.solve_ik(
                self.config_l,
                [self.task_l, self.posture_l],
                self.dt,
                self.solver,
                5e-3
            )
            self.config_l.integrate_inplace(vel, self.dt)

            # Update MuJoCo and (if sim) the viewer
            self.mj_data_l.qpos[:len(self.config_l.q)] = self.config_l.q
            mujoco.mj_forward(self.mj_model_l, self.mj_data_l)
            if self.simulation_only:
                # Update the full robot model directly for visualization
                if hasattr(self, 'mj_data_full') and self.mj_data_full is not None:
                    left_joint_ids = [21, 22, 23, 24, 25, 26, 27]
                    for i, joint_id in enumerate(left_joint_ids):
                        if i < len(self.config_l.q):
                            joint_name = f"j{joint_id}"
                            mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                            if mj_joint_id >= 0:
                                self.mj_data_full.qpos[mj_joint_id] = self.config_l.q[i]
                    
                    # Force forward kinematics and viewer update
                    mujoco.mj_forward(self.mj_model_full, self.mj_data_full)
                    if self.viewer_full:
                        self.viewer_full.sync()
                self.rate.sleep()

        # If real hardware, send the joint ticks
        if not self.simulation_only:
            q = self.config_l.q[:len(self.left_ids)]
            ticks = [self._rad2tick(angle) for angle in q]
            self.write_positions(self.left_ids, ticks)

    # ----------------------------------------------------------------------------
    # Predefined pose shortcuts
    # ----------------------------------------------------------------------------
    def home(self):    
        self.write_positions(list(self.poses['home'].keys()), list(self.poses['home'].values()))

    def prepose(self): 
        self.write_positions(list(self.poses['prepose'].keys()), list(self.poses['prepose'].values()))

    def pose1(self):   
        self.write_positions(list(self.poses['pose1'].keys()), list(self.poses['pose1'].values()))

    def pose2(self):   
        self.write_positions(list(self.poses['pose2'].keys()), list(self.poses['pose2'].values()))

    # ----------------------------------------------------------------------------
    # Recording and playback
    # ----------------------------------------------------------------------------
    def save_sequences(self):
        with open(self.sequence_file, 'w') as f:
            json.dump(self.saved_sequences, f)
        print(f"💾 All sequences saved to {self.sequence_file}")

    def load_sequences(self):
        if os.path.exists(self.sequence_file):
            with open(self.sequence_file, 'r') as f:
                data = json.load(f)
                # Convert all inner pose lists from JSON to int (if needed)
                self.saved_sequences = {
                    name: [list(map(int, pose)) for pose in poses]
                    for name, poses in data.items()
                }
            print(f"📂 Loaded {len(self.saved_sequences)} sequences from {self.sequence_file}")
        else:
            self.saved_sequences = {}

    def record_joint_poses(self, num_poses: int, sequence_name: str = None):
        print(f"\n📝 Recording {num_poses} poses. Turn off torque to move by hand.")
        recorded = []
        for i in range(num_poses):
            print(f"➡️ Pose {i+1}")
            self.torque_off_all()
            input("📍 Move robot, then press Enter to record...")
            self.torque_on_all()
            time.sleep(0.2)
            pose = [self.read_position(jid) for jid in self.right_ids + self.left_ids]
            recorded.append(pose)
            print(f"✅ Recorded pose {i+1}")
        if sequence_name:
            self.saved_sequences[sequence_name] = recorded
            self.save_sequences()
            print(f"💾 Saved sequence '{sequence_name}' with {len(recorded)} poses.")
        else:
            self.recorded_poses = recorded
            print("💾 Recorded to temporary buffer.")

    def play_saved_sequence(self, sequence_name: str, delay_sec=5.0):
        seq = self.saved_sequences.get(sequence_name)
        if not seq:
            print(f"❌ Sequence '{sequence_name}' not found.")
            return
        for i, pose in enumerate(seq):
            print(f"▶️ Playing pose {i+1}")
            self.write_positions(self.right_ids, pose[:len(self.right_ids)])
            self.write_positions(self.left_ids,  pose[len(self.right_ids):])
            time.sleep(delay_sec)
        print("✅ Playback complete.")

    def list_saved_sequences(self):
        print("\n📚 Saved Sequences:")
        for name, seq in self.saved_sequences.items():
            print(f" - {name}: {len(seq)} poses")

    # ----------------------------------------------------------------------------
    # Forward Kinematics (FK) Methods (NEW)
    # ----------------------------------------------------------------------------
    def perform_fk_right_arm(self, joint_values, input_format='ticks', update_simulation=False):
        """
        Perform Forward Kinematics for the right arm.
        
        Args:
            joint_values: List/array of 7 joint values for right arm joints (j11-j17)
            input_format: 'ticks' (default) or 'radians'
            update_simulation: If True, updates the simulation viewer to show the pose
            
        Returns:
            dict: {'position': [x, y, z], 'orientation': [w, x, y, z]}
        """
        if len(joint_values) != 7:
            raise ValueError("Right arm requires exactly 7 joint values")
        
        # Convert to radians if input is in ticks
        if input_format == 'ticks':
            joint_radians = [self._tick2rad(tick) for tick in joint_values]
        elif input_format == 'radians':
            joint_radians = list(joint_values)
        else:
            raise ValueError("input_format must be 'ticks' or 'radians'")
        
        # Perform forward kinematics
        temp_data = mujoco.MjData(self.mj_model_r)
        temp_data.qpos[:len(joint_radians)] = joint_radians
        mujoco.mj_forward(self.mj_model_r, temp_data)
        
        # Get end-effector body ID
        body_id = mujoco.mj_name2id(self.mj_model_r, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_2")
        if body_id < 0:
            raise ValueError("Body 'right_wrist_2' not found in model")
        
        # Extract position and orientation from MuJoCo data
        position = temp_data.xpos[body_id].copy()  # 3D position
        quat_xyzw = temp_data.xquat[body_id].copy()  # Quaternion in (w, x, y, z) format
        
        # Update simulation if requested
        if update_simulation and self.simulation_only:
            # Update the right arm configuration
            self.config_r.q[:len(joint_radians)] = joint_radians
            
            # Directly update right arm joints in full model
            if hasattr(self, 'mj_data_full') and self.mj_data_full is not None:
                right_joint_ids = [11, 12, 13, 14, 15, 16, 17]
                for i, joint_id in enumerate(right_joint_ids):
                    if i < len(joint_radians):
                        joint_name = f"j{joint_id}"
                        mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                        if mj_joint_id >= 0:
                            self.mj_data_full.qpos[mj_joint_id] = joint_radians[i]
                
                # Force forward kinematics and viewer update
                mujoco.mj_forward(self.mj_model_full, self.mj_data_full)
                if self.viewer_full:
                    self.viewer_full.sync()
        
        return {
            'position': position.tolist(),
            'orientation': quat_xyzw.tolist()
        }
    
    def perform_fk_left_arm(self, joint_values, input_format='ticks', update_simulation=False):
        """
        Perform Forward Kinematics for the left arm.
        
        Args:
            joint_values: List/array of 7 joint values for left arm joints (j21-j27)
            input_format: 'ticks' (default) or 'radians'
            update_simulation: If True, updates the simulation viewer to show the pose
            
        Returns:
            dict: {'position': [x, y, z], 'orientation': [w, x, y, z]}
        """
        if len(joint_values) != 7:
            raise ValueError("Left arm requires exactly 7 joint values")
        
        # Convert to radians if input is in ticks
        if input_format == 'ticks':
            joint_radians = [self._tick2rad(tick) for tick in joint_values]
        elif input_format == 'radians':
            joint_radians = list(joint_values)
        else:
            raise ValueError("input_format must be 'ticks' or 'radians'")
        
        # Perform forward kinematics
        temp_data = mujoco.MjData(self.mj_model_l)
        temp_data.qpos[:len(joint_radians)] = joint_radians
        mujoco.mj_forward(self.mj_model_l, temp_data)
        
        # Get end-effector body ID
        body_id = mujoco.mj_name2id(self.mj_model_l, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_2")
        if body_id < 0:
            raise ValueError("Body 'left_wrist_2' not found in model")
        
        # Extract position and orientation from MuJoCo data
        position = temp_data.xpos[body_id].copy()  # 3D position
        quat_xyzw = temp_data.xquat[body_id].copy()  # Quaternion in (w, x, y, z) format
        
        # Update simulation if requested
        if update_simulation and self.simulation_only:
            # Update the left arm configuration
            self.config_l.q[:len(joint_radians)] = joint_radians
            
            # Directly update left arm joints in full model
            if hasattr(self, 'mj_data_full') and self.mj_data_full is not None:
                left_joint_ids = [21, 22, 23, 24, 25, 26, 27]
                for i, joint_id in enumerate(left_joint_ids):
                    if i < len(joint_radians):
                        joint_name = f"j{joint_id}"
                        mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                        if mj_joint_id >= 0:
                            self.mj_data_full.qpos[mj_joint_id] = joint_radians[i]
                
                # Force forward kinematics and viewer update
                mujoco.mj_forward(self.mj_model_full, self.mj_data_full)
                if self.viewer_full:
                    self.viewer_full.sync()
        
        return {
            'position': position.tolist(),
            'orientation': quat_xyzw.tolist()
        }
    
    def perform_fk_both_arms(self, joint_values, input_format='ticks', update_simulation=False):
        """
        Perform Forward Kinematics for both arms.
        
        Args:
            joint_values: List/array of 14 joint values [right_arm_7_joints, left_arm_7_joints]
                         Order: [j11,j12,j13,j14,j15,j16,j17, j21,j22,j23,j24,j25,j26,j27]
            input_format: 'ticks' (default) or 'radians'
            update_simulation: If True, updates the simulation viewer to show the pose
            
        Returns:
            dict: {
                'right_arm': {'position': [x, y, z], 'orientation': [w, x, y, z]},
                'left_arm': {'position': [x, y, z], 'orientation': [w, x, y, z]}
            }
        """
        if len(joint_values) != 14:
            raise ValueError("Both arms require exactly 14 joint values (7 for each arm)")
        
        # Split joint values into right and left arms
        right_joints = joint_values[:7]  # First 7 joints for right arm
        left_joints = joint_values[7:]   # Last 7 joints for left arm
        
        # Perform FK for both arms
        right_result = self.perform_fk_right_arm(right_joints, input_format, update_simulation)
        left_result = self.perform_fk_left_arm(left_joints, input_format, update_simulation)
        
        return {
            'right_arm': right_result,
            'left_arm': left_result
        }
    
    def set_simulation_pose(self, joint_values, input_format='ticks', arm='both'):
        """
        Set the simulation robot to a specific joint configuration for visualization.
        
        Args:
            joint_values: Joint values (7 for single arm, 14 for both arms)
            input_format: 'ticks' (default) or 'radians'
            arm: 'right', 'left', or 'both' (default)
        """
        if not self.simulation_only:
            print("Warning: set_simulation_pose only works in simulation mode")
            return
            
        if not hasattr(self, 'mj_data_full') or self.mj_data_full is None:
            print("Warning: Full robot model not available")
            return
            
        # Convert to radians if needed
        if input_format == 'ticks':
            joint_radians = [self._tick2rad(tick) for tick in joint_values]
        else:
            joint_radians = list(joint_values)
            
        if arm == 'both':
            if len(joint_values) != 14:
                raise ValueError("Both arms require exactly 14 joint values")
            
            # Update both arm configurations for consistency
            self.config_r.q[:7] = joint_radians[:7]  # Right arm
            self.config_l.q[:7] = joint_radians[7:]  # Left arm
            
            # Directly update full model joints
            right_joint_ids = [11, 12, 13, 14, 15, 16, 17]
            left_joint_ids = [21, 22, 23, 24, 25, 26, 27]
            
            # Update right arm joints in full model
            for i, joint_id in enumerate(right_joint_ids):
                joint_name = f"j{joint_id}"
                mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if mj_joint_id >= 0:
                    self.mj_data_full.qpos[mj_joint_id] = joint_radians[i]
            
            # Update left arm joints in full model
            for i, joint_id in enumerate(left_joint_ids):
                joint_name = f"j{joint_id}"
                mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if mj_joint_id >= 0:
                    self.mj_data_full.qpos[mj_joint_id] = joint_radians[7 + i]
            
        elif arm == 'right':
            if len(joint_values) != 7:
                raise ValueError("Right arm requires exactly 7 joint values")
                
            self.config_r.q[:7] = joint_radians
            
            # Directly update right arm joints in full model
            right_joint_ids = [11, 12, 13, 14, 15, 16, 17]
            for i, joint_id in enumerate(right_joint_ids):
                joint_name = f"j{joint_id}"
                mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if mj_joint_id >= 0:
                    self.mj_data_full.qpos[mj_joint_id] = joint_radians[i]
                    
        elif arm == 'left':
            if len(joint_values) != 7:
                raise ValueError("Left arm requires exactly 7 joint values")
                
            self.config_l.q[:7] = joint_radians
            
            # Directly update left arm joints in full model
            left_joint_ids = [21, 22, 23, 24, 25, 26, 27]
            for i, joint_id in enumerate(left_joint_ids):
                joint_name = f"j{joint_id}"
                mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if mj_joint_id >= 0:
                    self.mj_data_full.qpos[mj_joint_id] = joint_radians[i]
        else:
            raise ValueError("arm must be 'right', 'left', or 'both'")
        
        # Force forward kinematics and viewer update
        mujoco.mj_forward(self.mj_model_full, self.mj_data_full)
        if self.viewer_full:
            self.viewer_full.sync()
        
        print(f"✅ Updated {arm} arm(s) in simulation")
        
        # Add a small delay to ensure the viewer updates
        time.sleep(0.1)
        
    def debug_joint_info(self):
        """Debug function to check available joints in the full model."""
        if not self.simulation_only or not hasattr(self, 'mj_model_full'):
            print("Only available in simulation mode")
            return
            
        print("🔍 Available joints in full model:")
        for i in range(self.mj_model_full.njnt):
            joint_name = mujoco.mj_id2name(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name:
                print(f"  Joint {i}: {joint_name}")
        
        print(f"\n📊 Model info:")
        print(f"  Total joints: {self.mj_model_full.njnt}")
        print(f"  Total DOF: {self.mj_model_full.nv}")
        print(f"  qpos size: {len(self.mj_data_full.qpos)}")
        
        # Check specific arm joints
        for arm, joint_ids in [("Right", [11,12,13,14,15,16,17]), ("Left", [21,22,23,24,25,26,27])]:
            print(f"\n{arm} arm joints:")
            for joint_id in joint_ids:
                joint_name = f"j{joint_id}"
                mj_joint_id = mujoco.mj_name2id(self.mj_model_full, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                status = "✅ Found" if mj_joint_id >= 0 else "❌ Not found"
                print(f"  {joint_name}: {status} (mj_id: {mj_joint_id})")
    
    def get_current_fk(self):
        """
        Get Forward Kinematics results for current joint positions.
        Reads current joint positions from hardware and computes FK.
        
        Returns:
            dict: {
                'right_arm': {'position': [x, y, z], 'orientation': [w, x, y, z]},
                'left_arm': {'position': [x, y, z], 'orientation': [w, x, y, z]}
            }
        """
        if self.simulation_only:
            # Use current configuration values in simulation
            right_result = self.perform_fk_right_arm([self._rad2tick(q) for q in self.config_r.q[:7]], 'ticks')
            left_result = self.perform_fk_left_arm([self._rad2tick(q) for q in self.config_l.q[:7]], 'ticks')
        else:
            # Read current positions from hardware
            current_positions = self.read_all_positions()
            right_joints = [current_positions[jid] for jid in self.right_ids]
            left_joints = [current_positions[jid] for jid in self.left_ids]
            
            right_result = self.perform_fk_right_arm(right_joints, 'ticks')
            left_result = self.perform_fk_left_arm(left_joints, 'ticks')
        
        return {
            'right_arm': right_result,
            'left_arm': left_result
        }