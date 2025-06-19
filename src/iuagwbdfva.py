import os
import time
import math
import numpy as np
import serial.tools.list_ports
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite
import json

# IK solver imports - multiple options
try:
    from ikpy.chain import Chain
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False
    print("Warning: ikpy not available")

try:
    import trac_ik_python.trac_ik as trac_ik
    TRACIK_AVAILABLE = True
except ImportError:
    TRACIK_AVAILABLE = False
    print("Warning: trac_ik not available")

try:
    import kdl_python as kdl
    KDL_AVAILABLE = True
except ImportError:
    KDL_AVAILABLE = False
    print("Warning: KDL not available")

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("Warning: Pinocchio not available")

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

class IKSolver:
    """Base class for different IK solver implementations"""
    def __init__(self, urdf_file, base_link, tip_link, joint_names):
        self.urdf_file = urdf_file
        self.base_link = base_link
        self.tip_link = tip_link
        self.joint_names = joint_names
        self.setup_solver()
    
    def setup_solver(self):
        raise NotImplementedError
    
    def solve_ik(self, target_pos, target_quat=None, seed_angles=None):
        raise NotImplementedError

class TracIKSolver(IKSolver):
    """TRAC-IK solver implementation"""
    def setup_solver(self):
        if not TRACIK_AVAILABLE:
            raise ImportError("trac_ik_python not available")
        
        # Joint limits (you may need to adjust these based on your robot)
        lower_limits = [-3.14159] * len(self.joint_names)  # -180 degrees
        upper_limits = [3.14159] * len(self.joint_names)   # +180 degrees
        
        self.solver = trac_ik.IK(
            self.base_link,
            self.tip_link, 
            self.urdf_file,
            timeout=0.005,  # 5ms timeout
            epsilon=1e-5,   # precision
            solve_type="Speed"  # or "Distance" or "Manipulation1" or "Manipulation2"
        )
        
    def solve_ik(self, target_pos, target_quat=None, seed_angles=None):
        if seed_angles is None:
            seed_angles = [0.0] * len(self.joint_names)
        
        if target_quat is None:
            # Default orientation (pointing down)
            target_quat = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]
        
        # Convert to the format expected by TRAC-IK
        result = self.solver.get_ik(
            seed_angles,
            target_pos[0], target_pos[1], target_pos[2],  # position
            target_quat[0], target_quat[1], target_quat[2], target_quat[3]  # orientation
        )
        
        return result if result is not None else []

class KDLSolver(IKSolver):
    """KDL solver implementation"""
    def setup_solver(self):
        if not KDL_AVAILABLE:
            raise ImportError("PyKDL not available")
        
        # You'll need to manually construct the KDL chain
        # This is a simplified example - you'd need to parse the URDF
        self.chain = kdl.Chain()
        # Add joints and segments based on your URDF
        # This requires manual setup based on your robot structure
        
    def solve_ik(self, target_pos, target_quat=None, seed_angles=None):
        # Implement KDL IK solving
        # This is more complex and requires setting up the chain properly
        pass

class PinocchioSolver(IKSolver):
    """Pinocchio solver implementation"""
    def setup_solver(self):
        if not PINOCCHIO_AVAILABLE:
            raise ImportError("Pinocchio not available")
        
        self.model = pin.buildModelFromUrdf(self.urdf_file)
        self.data = self.model.createData()
        
        # Get frame ID for end effector
        self.frame_id = self.model.getFrameId(self.tip_link)
        
    def solve_ik(self, target_pos, target_quat=None, seed_angles=None):
        if seed_angles is None:
            seed_angles = pin.neutral(self.model)
        else:
            seed_angles = np.array(seed_angles)
        
        # Create target SE3 transform
        if target_quat is None:
            target_rotation = np.eye(3)
        else:
            # Convert quaternion to rotation matrix
            from scipy.spatial.transform import Rotation
            target_rotation = Rotation.from_quat(target_quat).as_matrix()
        
        target_transform = pin.SE3(target_rotation, np.array(target_pos))
        
        # Solve IK using Pinocchio's solver
        q_result = pin.computeInverseKinematics(
            self.model, self.data, self.frame_id, target_transform, seed_angles
        )
        
        return q_result.tolist()

class ImprovedIKPySolver(IKSolver):
    """Improved ikpy solver with better configuration"""
    def setup_solver(self):
        if not IKPY_AVAILABLE:
            raise ImportError("ikpy not available")
        
        # Load with better configuration
        self.chain = Chain.from_urdf_file(
            self.urdf_file,
            base_elements=[self.base_link],
            last_link_vector=None,
            active_links_mask=None  # Let ikpy determine automatically
        )
        
        # Alternative: manually create chain if automatic doesn't work
        # You might need to adjust the active links mask based on your robot
        
    def solve_ik(self, target_pos, target_quat=None, seed_angles=None):
        # Convert target to homogeneous transformation matrix
        if target_quat is None:
            # Default orientation matrix (identity)
            target_orientation = np.eye(3)
        else:
            # Convert quaternion to rotation matrix
            from scipy.spatial.transform import Rotation
            target_orientation = Rotation.from_quat(target_quat).as_matrix()
        
        # Create 4x4 transformation matrix
        target_transform = np.eye(4)
        target_transform[:3, :3] = target_orientation
        target_transform[:3, 3] = target_pos
        
        # Solve IK with initial position if provided
        if seed_angles is not None:
            # Pad with zeros for fixed joints
            padded_seed = [0.0] + list(seed_angles) + [0.0]
            result = self.chain.inverse_kinematics(
                target_transform, 
                initial_position=padded_seed
            )
        else:
            result = self.chain.inverse_kinematics(target_transform)
        
        # Extract only the active joint angles (remove fixed joints)
        active_joints = []
        for i, link in enumerate(self.chain.links):
            if i > 0 and i < len(self.chain.links) - 1:  # Skip base and end effector
                if hasattr(link, 'bounds') and link.bounds is not None:
                    active_joints.append(result[i])
        
        return active_joints

class DynamixelRobot:
    def __init__(
        self,
        device_name: str = '',
        baud_rate: int = 4000000,
        protocol_version: float = 2.0,
        right_urdf: str = 'urdf/right_arm_URDF.urdf',
        left_urdf:  str = 'urdf/left_arm_URDF.urdf',
        ik_solver_type: str = 'tracik'  # 'tracik', 'pinocchio', 'ikpy_improved', 'kdl'
    ):

        # Auto-detect or verify serial port
        if not device_name:
            ports = list_available_ports()
            if not ports:
                raise IOError("No available serial ports found.")
            print("Available ports:", ports)
            device_name = ports[0]
            print(f"Auto-selecting port: {device_name}")

        # Initialize Port & Packet handlers
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

        # Enable torque on all joints by default
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

        # Initialize IK solvers
        self.ik_solver_type = ik_solver_type
        self.setup_ik_solvers(right_urdf, left_urdf)

    def setup_ik_solvers(self, right_urdf, left_urdf):
        """Initialize IK solvers for both arms"""
        
        # Joint names for each arm (adjust based on your URDF)
        right_joint_names = ['j11', 'j12', 'j13', 'j14', 'j15', 'j16', 'j17']
        left_joint_names = ['j21', 'j22', 'j23', 'j24', 'j25', 'j26', 'j27']
        
        try:
            if self.ik_solver_type == 'tracik':
                self.right_ik = TracIKSolver(right_urdf, 'base_link', 'j17', right_joint_names)
                self.left_ik = TracIKSolver(left_urdf, 'base_link', 'j27', left_joint_names)
            elif self.ik_solver_type == 'pinocchio':
                self.right_ik = PinocchioSolver(right_urdf, 'base_link', 'j17', right_joint_names)
                self.left_ik = PinocchioSolver(left_urdf, 'base_link', 'j27', left_joint_names)
            elif self.ik_solver_type == 'ikpy_improved':
                self.right_ik = ImprovedIKPySolver(right_urdf, 'base_link', 'j17', right_joint_names)
                self.left_ik = ImprovedIKPySolver(left_urdf, 'base_link', 'j27', left_joint_names)
            elif self.ik_solver_type == 'kdl':
                self.right_ik = KDLSolver(right_urdf, 'base_link', 'j17', right_joint_names)
                self.left_ik = KDLSolver(left_urdf, 'base_link', 'j27', left_joint_names)
            else:
                raise ValueError(f"Unknown IK solver type: {self.ik_solver_type}")
                
            print(f"✅ IK solver '{self.ik_solver_type}' initialized successfully")
            
        except Exception as e:
            print(f"❌ Failed to initialize {self.ik_solver_type} solver: {e}")
            print("Falling back to basic ikpy solver...")
            # Fallback to basic ikpy
            if IKPY_AVAILABLE:
                self.setup_basic_ikpy(right_urdf, left_urdf)
            else:
                print("❌ No IK solver available!")
                self.right_ik = None
                self.left_ik = None

    def setup_basic_ikpy(self, right_urdf, left_urdf):
        """Fallback basic ikpy setup"""
        try:
            # Basic ikpy setup (your original code)
            right_full = Chain.from_urdf_file(right_urdf)
            r_names = [l.name for l in right_full.links]
            r_start, r_end = r_names.index('j11'), r_names.index('j17')
            right_links = right_full.links[r_start:r_end+2]
            active_r = [True]*(len(right_links)-1) + [False]

            left_full = Chain.from_urdf_file(left_urdf)
            l_names = [l.name for l in left_full.links]
            l_start, l_end = l_names.index('j21'), l_names.index('j27')
            left_links = left_full.links[l_start:l_end+2]
            active_l = [True]*(len(left_links)-1) + [False]

            self.chain_r = Chain(name='right_arm', links=right_links, active_links_mask=active_r)
            self.chain_l = Chain(name='left_arm',  links=left_links,  active_links_mask=active_l)
            self.ik_solver_type = 'basic_ikpy'
            print("✅ Basic ikpy fallback initialized")
        except Exception as e:
            print(f"❌ Even basic ikpy failed: {e}")
            self.chain_r = None
            self.chain_l = None

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

    def move_right_hand_cartesian(self, x, y, z, quat=None, seed_angles=None):
        """Move right hand to cartesian position with improved IK"""
        if self.right_ik is None:
            print("❌ No IK solver available for right arm")
            return False
        
        try:
            if self.ik_solver_type == 'basic_ikpy':
                # Use original ikpy method
                angles = self.chain_r.inverse_kinematics([x,y,z])[1:1+len(self.right_ids)]
            else:
                # Use advanced IK solver
                angles = self.right_ik.solve_ik([x, y, z], quat, seed_angles)
            
            if angles and len(angles) >= len(self.right_ids):
                ticks = [self._rad2tick(a) for a in angles[:len(self.right_ids)]]
                self.write_positions(self.right_ids, ticks)
                return True
            else:
                print("❌ IK solution not found for right arm")
                return False
                
        except Exception as e:
            print(f"❌ IK error for right arm: {e}")
            return False

    def move_left_hand_cartesian(self, x, y, z, quat=None, seed_angles=None):
        """Move left hand to cartesian position with improved IK"""
        if self.left_ik is None:
            print("❌ No IK solver available for left arm")
            return False
        
        try:
            if self.ik_solver_type == 'basic_ikpy':
                # Use original ikpy method
                angles = self.chain_l.inverse_kinematics([x,y,z])[1:1+len(self.left_ids)]
            else:
                # Use advanced IK solver
                angles = self.left_ik.solve_ik([x, y, z], quat, seed_angles)
            
            if angles and len(angles) >= len(self.left_ids):
                ticks = [self._rad2tick(a) for a in angles[:len(self.left_ids)]]
                self.write_positions(self.left_ids, ticks)
                return True
            else:
                print("❌ IK solution not found for left arm")
                return False
                
        except Exception as e:
            print(f"❌ IK error for left arm: {e}")
            return False

    def get_current_cartesian_position(self, arm='right'):
        """Get current cartesian position of the specified arm"""
        if arm == 'right' and hasattr(self, 'chain_r') and self.chain_r:
            current_angles = [self._tick2rad(self.read_position(jid)) for jid in self.right_ids]
            # Add padding for fixed joints
            padded_angles = [0.0] + current_angles + [0.0]
            transform = self.chain_r.forward_kinematics(padded_angles)
            return transform[:3, 3]  # Extract position
        elif arm == 'left' and hasattr(self, 'chain_l') and self.chain_l:
            current_angles = [self._tick2rad(self.read_position(jid)) for jid in self.left_ids]
            # Add padding for fixed joints
            padded_angles = [0.0] + current_angles + [0.0]
            transform = self.chain_l.forward_kinematics(padded_angles)
            return transform[:3, 3]  # Extract position
        else:
            print(f"❌ Cannot get cartesian position for {arm} arm")
            return None

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
    # IK Testing and Debugging
    # ----------------------------------------------------------------------------
    def test_ik_solver(self, arm='right', test_positions=None):
        """Test the IK solver with known positions"""
        if test_positions is None:
            # Default test positions (adjust based on your robot's workspace)
            test_positions = [
                [0.3, 0.0, 0.3],   # Forward
                [0.2, 0.2, 0.3],   # Forward-right
                [0.2, -0.2, 0.3],  # Forward-left
                [0.25, 0.0, 0.5],  # Up
                [0.25, 0.0, 0.1],  # Down
            ]
        
        print(f"\n🧪 Testing IK solver for {arm} arm:")
        
        for i, pos in enumerate(test_positions):
            print(f"\nTest {i+1}: Target position {pos}")
            
            if arm == 'right':
                success = self.move_right_hand_cartesian(pos[0], pos[1], pos[2])
            else:
                success = self.move_left_hand_cartesian(pos[0], pos[1], pos[2])
            
            if success:
                time.sleep(2)  # Wait for movement
                actual_pos = self.get_current_cartesian_position(arm)
                if actual_pos is not None:
                    error = np.linalg.norm(np.array(pos) - np.array(actual_pos))
                    print(f"✅ Success! Actual: {actual_pos}, Error: {error:.4f}m")
                else:
                    print("✅ Movement completed (position verification unavailable)")
            else:
                print("❌ Failed to find IK solution")
        
        print("\n🏠 Returning to home position...")
        self.home()

# Example usage and testing
if __name__ == "__main__":
    # Initialize robot with different IK solvers
    try:
        # Try TRAC-IK first (recommended)
        robot = DynamixelRobot(ik_solver_type='tracik')
    except:
        try:
            # Fallback to Pinocchio
            robot = DynamixelRobot(ik_solver_type='pinocchio')
        except:
            # Final fallback to improved ikpy
            robot = DynamixelRobot(ik_solver_type='ikpy_improved')
    
    # Test the IK solver
    robot.test_ik_solver('right')
    
    # Example usage with orientation control
    robot.move_right_hand_cartesian(
        x=0.3, y=0.0, z=0.3, 
        quat=[0, 0, 0, 1],  # No rotation
        seed_angles=[0, 0, 0, 0, 0, 0, 0]  # Starting guess
    )