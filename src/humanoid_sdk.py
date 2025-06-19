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

        # Load MuJoCo models for both arms
        self.mj_model_r = mujoco.MjModel.from_xml_path("/home/kptal/actuator_sdk/urdf/right_arm.xml")
        self.mj_data_r = mujoco.MjData(self.mj_model_r)

        self.mj_model_l = mujoco.MjModel.from_xml_path("/home/kptal/actuator_sdk/urdf/left_arm.xml")
        self.mj_data_l = mujoco.MjData(self.mj_model_l)

        if self.simulation_only:
            self.viewer_r = mujoco.viewer.launch_passive(self.mj_model_r, self.mj_data_r,show_left_ui=False,show_right_ui=False)
            self.viewer_l = mujoco.viewer.launch_passive(self.mj_model_l, self.mj_data_l,show_left_ui=False,show_right_ui=False)


        # Configuration
        self.config_r = mink.Configuration(self.mj_model_r)
        self.config_l = mink.Configuration(self.mj_model_l)


        self.task_r = mink.FrameTask(
            frame_name="right_wrist_2",  # or whatever your end-effector body is
            frame_type="body",
            position_cost=1.0,
            orientation_cost=0.0,
            lm_damping=1.0,
        )
        self.task_l = mink.FrameTask(
            frame_name="left_wrist_2",
            frame_type="body",
            position_cost=1.0,
            orientation_cost=0.0,
            lm_damping=1.0,
        )
        self.posture_r = mink.PostureTask(model=self.mj_model_r, cost=1e-2)
        self.posture_l = mink.PostureTask(model=self.mj_model_l, cost=1e-2)

        self.solver = "osqp"
        self.rate = RateLimiter(frequency=40.0, warn=False)
        self.max_iters = 20
        self.pos_threshold = 0.005
        self.dt = 1.0 / 40.0



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

    # def move_right_hand_cartesian(self, x, y, z):
    #     angles = self.chain_r.inverse_kinematics([x,y,z])[1:1+len(self.right_ids)]
    #     ticks  = [self._rad2tick(a) for a in angles]
    #     self.write_positions(self.right_ids, ticks)

    # def move_left_hand_cartesian(self, x, y, z):
    #     angles = self.chain_l.inverse_kinematics([x,y,z])[1:1+len(self.left_ids)]
    #     ticks  = [self._rad2tick(a) for a in angles]
    #     self.write_positions(self.left_ids, ticks)

    def move_right_hand_cartesian(self, x, y, z):
        T_target = mink.SE3(wxyz_xyz=[0, 0, 0, 1, x, y, z])
        self.task_r.set_target(T_target)
        self.posture_r.set_target_from_configuration(self.config_r)

        for i in range(self.max_iters):
            vel = mink.solve_ik(self.config_r, [self.task_r, self.posture_r], self.dt, self.solver, 5e-3)
            self.config_r.integrate_inplace(vel, self.dt)

            # ✅ Update MuJoCo qpos
            self.mj_data_r.qpos[:len(self.config_r.q)] = self.config_r.q
            mujoco.mj_forward(self.mj_model_r, self.mj_data_r)

            print(f"Step {i}, vel norm: {np.linalg.norm(vel)}")

            if self.simulation_only:
                self.viewer_r.sync()
                self.rate.sleep()

        if not self.simulation_only:
            q = self.config_r.q[:len(self.right_ids)]
            self.write_positions(self.right_ids, [self._rad2tick(angle) for angle in q])


    def move_left_hand_cartesian(self, x, y, z):
        T_target = mink.SE3(wxyz_xyz=[0, 0, 0, 1, x, y, z])
        self.task_l.set_target(T_target)
        self.posture_l.set_target_from_configuration(self.config_l)

        for i in range(self.max_iters):
            vel = mink.solve_ik(self.config_l, [self.task_l, self.posture_l], self.dt, self.solver, 5e-3)
            self.config_l.integrate_inplace(vel, self.dt)

            # ✅ Update MuJoCo qpos
            self.mj_data_l.qpos[:len(self.config_l.q)] = self.config_l.q
            mujoco.mj_forward(self.mj_model_l, self.mj_data_l)

            print(f"Step {i}, vel norm: {np.linalg.norm(vel)}")

            if self.simulation_only:
                self.viewer_l.sync()
                self.rate.sleep()

        if not self.simulation_only:
            q = self.config_l.q[:len(self.left_ids)]
            self.write_positions(self.left_ids, [self._rad2tick(angle) for angle in q])




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

