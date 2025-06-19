#!/usr/bin/env python3
import time
from dynamixel_sdk import PortHandler, PacketHandler
from humanoid_sdk import DynamixelRobot  # adjust import as needed

# Control table address to disable torque
ADDR_TORQUE_ENABLE = 64
TORQUE_DISABLE     = 0

def test_cartesian_hands():
    device = "/dev/ttyUSB0"
    # Instantiate robot (requires valid URDFs for both arms)
    robot = DynamixelRobot(
        device_name=device,
        baud_rate=4000000
    )

    # Use conservative motion profiles
    robot.set_all_profiles(acceleration=50, velocity=15)

    try:
        # 1) Safe starting stance (home)
        print("-> Moving to home pose")
        robot.home()
        time.sleep(5.0)


        # robot.close_left_hand_gripper()
        # robot.close_right_hand_gripper()
        # time.sleep(2.0)

        # # 2) Move both hands to Cartesian origin (0,0,0)
        # target = [0, 0, 0]
        # print(f"-> Moving right hand to {target}")
        # robot.move_right_hand_cartesian(*target)
        # print(f"-> Moving left hand to {target}")
        # robot.move_left_hand_cartesian(*target)
        # time.sleep(4.0)

        # 3) Print joint angles at the Cartesian zero position
        # print("-> Joint angles at Cartesian [0, 0, 0]")
        # robot.print_joint_angles()
        # time.sleep(18.0)

        # 4) Return to home
        print("-> Returning to home pose")
        robot.home()
        time.sleep(2.0)

    except Exception as e:
        print(f"[ERROR] {e}")

    finally:
        # Disable torque on all motors
        print("-> Disabling torque on all joints")
        for dxl_id in robot.all_ids:
            robot.packet.write1ByteTxRx(
                robot.port, dxl_id,
                ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )
        robot.port.closePort()
        print("-> Test complete.")

if __name__ == "__main__":
    test_cartesian_hands()




