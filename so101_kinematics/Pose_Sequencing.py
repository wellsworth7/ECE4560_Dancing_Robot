import time
from so101_utils import load_calibration, setup_motors, move_to_pose

PORT_ID = "COM6"  # Replace with your actual port
ROBOT_NAME = "follower_arm"

calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)



def Pose1():
    """Pointing Ahead"""
    angles = {
        'shoulder_pan': 45.351648351648351,
        'shoulder_lift': -69.4065934065934,
        'elbow_flex': 59.34065934065934,
        'wrist_flex': -59.64835164835165,
        'wrist_roll': -12.043956043956044,
        'gripper': 6.688102893890675
    }
    print("\nPose #1: Pointing Ahead")
    move_to_pose(bus, angles, duration=1)


def Pose2():
    """Pointing To The Sky"""
    angles = {
        'shoulder_pan': -45.615384615384615,
        'shoulder_lift': -63.86813186813187,
        'elbow_flex': -53.0989010989011,
        'wrist_flex': 14.197802197802197,
        'wrist_roll': -11.252747252747254,
        'gripper': 85.688102893890675
    }
    print("\nPose #2: Pointing To The Sky")
    move_to_pose(bus, angles, duration=2)


def Pose3():
    """Pointing Down"""
    angles = {
        'shoulder_pan': 45.4175824175824174,
        'shoulder_lift': -35.38461538461539,
        'elbow_flex': -0.3516483516483517,
        'wrist_flex': 88.21978021978022,
        'wrist_roll': -4.21978021978022,
        'gripper': 1.929260450160772
    }
    print("\nPose #3: Pointing Down")
    move_to_pose(bus, angles, duration=2)

def Pose4():
    """Pointing Straight"""
    angles = {
        'shoulder_pan': -45.4175824175824174,
        'shoulder_lift': -35.56043956043956,
        'elbow_flex': -1.054945054945055,
        'wrist_flex': -1.4505494505494505,
        'wrist_roll': -3.6043956043956045,
        'gripper': 85.929260450160772
    }
    print("\nPose #6: Pointing Straight")
    move_to_pose(bus, angles, duration=1)


def Pose5():
    """Pointing Up"""
    angles = {
        'shoulder_pan': 45.4175824175824174,
        'shoulder_lift': -35.472527472527474,
        'elbow_flex': -0.43956043956043955,
        'wrist_flex': -92.08791208791209,
        'wrist_roll': -4.21978021978022,
        'gripper': 1.929260450160772
    }
    print("\nPose #7: Pointing Up")
    move_to_pose(bus, angles, duration=1)



def Pose6():
    """Bending Backwards"""
    angles = {
        'shoulder_pan': -45.6483516483516483,
        'shoulder_lift': -94.1978021978022,
        'elbow_flex': -106.37362637362638,
        'wrist_flex': -101.23076923076923,
        'wrist_roll': -4.3076923076923075,
        'gripper': 85.929260450160772
    }
    print("\nPose #4: Bending Backwards")
    move_to_pose(bus, angles, duration=2)


def Pose7():
    """Bent Forward"""
    angles = {
        'shoulder_pan': 45.6483516483516483,
        'shoulder_lift': 8.483516483516484,
        'elbow_flex': 20.65934065934066,
        'wrist_flex': -103.25274725274726,
        'wrist_roll': -3.78021978021978,
        'gripper': 1.929260450160772
    }
    print("\nPose #5: Bent Forward")
    move_to_pose(bus, angles, duration=2)

def Pose8():
    """Diagonal Straight"""
    angles = {
        'shoulder_pan': -1.1868131868131868,
        'shoulder_lift': 24.21978021978022,
        'elbow_flex': -92.57142857142857,
        'wrist_flex': -1.010989010989011,
        'wrist_roll': -3.956043956043956,
        'gripper': 85.315112540192926
    }
    print("\nPose #8: Diagonal Straight")
    move_to_pose(bus, angles, duration=2)

def Pose9():
    """Diagonal Down"""
    angles = {
        'shoulder_pan': -0.5714285714285714,
        'shoulder_lift': 22.285714285714285,
        'elbow_flex': -106.02197802197803,
        'wrist_flex': -89.27472527472527,
        'wrist_roll': -4.571428571428571,
        'gripper': 2.315112540192926
    }
    print("\nPose #9: Diagonal Down")
    move_to_pose(bus, angles, duration=2)

def Pose10():
    """Diagonal Up"""
    angles = {
        'shoulder_pan': -0.4835164835164835,
        'shoulder_lift': 14.197802197802197,
        'elbow_flex': -90.81318681318682,
        'wrist_flex': 101.14285714285714,
        'wrist_roll': -4.835164835164835,
        'gripper': 85.315112540192926
    }
    print("\nPose #10: Diagonal Up")
    move_to_pose(bus, angles, duration=2)



print("\n--- Starting Pose Sequence ---")

Pose1()
Pose2()
Pose3()
Pose4()
Pose5()
Pose6()
Pose7()
Pose8()
Pose9()
Pose10()


print("\nAll poses complete! Disabling torque.")
bus.disable_torque()
