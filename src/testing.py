import time
from humanoid_sdk import DynamixelRobot  # Replace with your actual module name

def main():
    print("🤖 Initializing Robot...")
    robot = DynamixelRobot("/dev/cu.usbserial-FT88Z0EM")  # Update port if needed
    robot.set_all_profiles(acceleration=50, velocity=20)

    print("\n🏠 Moving to home pose...")
    robot.prepose()
    time.sleep(4)

    # print("\n🚶 Moving to prepose...")
    # robot.prepose()
    # time.sleep(4)

    # print("\n🕺 Pose 1 - Dance start!")
    # robot.pose1()
    # time.sleep(4)

    # print("\n💃 Pose 2 - Dance end!")
    # robot.pose2()
    # time.sleep(4)

    # print("\n🎯 Moving RIGHT arm in joint space...")
    # robot.move_right_hand_joints(1600, 1700, 1800, 1900, 2000, 2100, 2200)
    # time.sleep(4)

    # print("\n🎯 Moving LEFT arm in joint space...")
    # robot.move_left_hand_joints(2200, 2100, 2000, 1900, 1800, 1700, 1600)
    # time.sleep(4)

    print("\n🧭 Moving RIGHT arm in Cartesian space...")
    robot.move_right_hand_cartesian(0.021,-0.355,-0.666)
    time.sleep(5)

    print("\n🧭 Moving LEFT arm in Cartesian space...")
    robot.move_left_hand_cartesian(0.021,0.355,-0.666)
    time.sleep(5)

    # print("\n✋ Opening RIGHT gripper...")
    # robot.open_right_hand_gripper()
    # time.sleep(3)

    # print("\n🤏 Closing RIGHT gripper...")
    # robot.close_right_hand_gripper()
    # time.sleep(3)

    # print("\n✋ Opening LEFT gripper...")
    # robot.open_left_hand_gripper()
    # time.sleep(3)

    # print("\n🤏 Closing LEFT gripper...")
    # robot.close_left_hand_gripper()
    # time.sleep(3)

    # print("\n Let's record a motion sequence!")
    # robot.record_joint_poses(num_poses=2, sequence_name="student_wave")

    # print("\n📚 Listing saved sequences:")
    # robot.list_saved_sequences()

    # print("\n🔁 Playing back 'student_wave' sequence...")
    # robot.play_saved_sequence("student_wave")

    # print("\n🔁 Returning to home pose...")
    # robot.home()
    # time.sleep(4)

    print("\n✅ Demo complete! Robot is ready.")

    # Optionally disable torque at the end:
    # for dxl_id in robot.all_ids:
    #     robot.packet.write1ByteTxRx(robot.port, dxl_id, 64, 0)

if __name__ == "__main__":
    main()
