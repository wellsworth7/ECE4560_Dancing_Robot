# so101-utils.py
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from pathlib import Path
import draccus
import time

def load_calibration(ROBOT_NAME: str, calib_dir: str = None):
    """
    Helper to load calibration data from the specified file.

    Args:
        ROBOT_NAME (str): The robot name (e.g., "so101_follower")
        calib_dir (str, optional): Directory containing calibration JSON files.
            If None, defaults to 'calibration_files' inside this repo.

    Returns:
        dict[str, MotorCalibration]: Loaded calibration data.
    """
    # Get base directory of the current repo (two levels up from this file)
    repo_root = Path(__file__).resolve().parents[1]

    # Default calibration directory if not provided
    if calib_dir is None:
        calib_dir = repo_root / "so101_kinematics" / "calibration_files"
    else:
        calib_dir = Path(calib_dir).resolve()

    fpath = calib_dir / f"{ROBOT_NAME}.json"
    if not fpath.exists():
        raise FileNotFoundError(f"Calibration file not found: {fpath}")

    with open(fpath) as f, draccus.config_type("json"):
        calibration = draccus.load(dict[str, MotorCalibration], f)
    return calibration


def setup_motors(calibration, PORT_ID):
    norm_mode_body = MotorNormMode.DEGREES
    bus = FeetechMotorsBus(
                port=PORT_ID,
                motors={
                    "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                    "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                    "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                    "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                    "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
                },
                calibration=calibration,
            )
    bus.connect(True)

    with bus.torque_disabled():
        bus.configure_motors()
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            bus.write("P_Coefficient", motor, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            bus.write("I_Coefficient", motor, 0)
            bus.write("D_Coefficient", motor, 32) 
    return bus
def offset_config(config):
    offset_dict = config.copy()
    offset_dict['shoulder_pan'] -= 0
    offset_dict['shoulder_lift'] -= 0
    offset_dict['elbow_flex'] -= 0
    offset_dict['wrist_flex'] -= 0
    offset_dict['wrist_roll'] -= 0
    offset_dict['gripper'] -= 0
    return offset_dict

def move_to_pose(bus, desired_position, duration):
    start_time = time.time()
    starting_pose = bus.sync_read("Present_Position")
    
    while True:
        t = time.time() - start_time
        if t > duration:
            break

        # Interpolation factor [0,1] (make sure it doesn't exceed 1)
        alpha = min(t / duration, 1)

        # Interpolate each joint
        position_dict = {}
        for joint in desired_position:
            p0 = starting_pose[joint]
            pf = desired_position[joint]
            position_dict[joint] = (1 - alpha) * p0 + alpha * pf

        # Send command
        offset_dict = offset_config(position_dict)
        bus.sync_write("Goal_Position", position_dict, normalize=True)

        # (Optional) Read back
        present_pos = bus.sync_read("Present_Position")
        # print(present_pos)

        time.sleep(0.02)  # 50 Hz loop
    
def hold_position(bus, duration):
    current_pos = bus.sync_read("Present_Position")
    start_time = time.time()
    while True:
        t = time.time() - start_time
        if t > duration:
            break
        bus.sync_write("Goal_Position", current_pos, normalize=True)
        time.sleep(0.02)  # 50 Hz loop
