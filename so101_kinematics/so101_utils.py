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

def move_to_pose_cubic(bus, start_position, desired_position, duration):
    """
    Moves the robot from start_position to desired_position using cubic interpolation.
    """
    start_time = time.perf_counter()
    a0, a1, a2, a3 = {}, {}, {}, {}
    for joint in desired_position:
        p0 = start_position[joint]
        pf = desired_position[joint]
        v0 = 0.0  # start velocity
        vf = 0.0  # end velocity
        T = duration
        a0[joint] = p0
        a1[joint] = v0
        a2[joint] = (3 * (pf - p0) / (T**2)) - (2 * v0 + vf) / T
        a3[joint] = (-2 * (pf - p0) / (T**3)) + (v0 + vf) / (T**2)

    def cubic_interpolation(t, joint):
        """Returns position at time t for a joint using cubic interpolation"""
        tlim = min(max(t, 0.0), duration)
        pos = a0[joint] + a1[joint]*tlim + a2[joint]*(tlim**2) + a3[joint]*(tlim**3)
        return pos

    while True:
        t = time.perf_counter() - start_time
        if t > duration:
            break

        position_dict = {}
        for joint in desired_position:
            position_dict[joint] = cubic_interpolation(t, joint)
        # Send positions to hardware
        offset_dict = offset_config(position_dict)
        bus.sync_write("Goal_Position", offset_dict, normalize=True)

        time.sleep(0.02)  # 50 Hz loop

    # Ensure final position is exactly reached
    final_offset = offset_config(desired_position)
    bus.sync_write("Goal_Position", final_offset, normalize=True)

# -----------------------------
# SMOOTH CUBIC INTERPOLATION FUNCTION
# -----------------------------
def move_to_pose_cubic_cont(bus, start_position, desired_position, duration, v0_dict=None, v1_dict=None):
    """Cubic interpolation with optional nonzero boundary velocities per joint."""
    start_time = time.perf_counter()
    a0, a1, a2, a3 = {}, {}, {}, {}
    v0_dict = v0_dict or {joint: 0.0 for joint in desired_position}
    v1_dict = v1_dict or {joint: 0.0 for joint in desired_position}

    for joint in desired_position:
        p0 = start_position[joint]
        pf = desired_position[joint]
        v0 = v0_dict[joint]
        vf = v1_dict[joint]
        T = duration

        a0[joint] = p0
        a1[joint] = v0
        a2[joint] = (3*(pf - p0)/(T**2)) - (2*v0 + vf)/T
        a3[joint] = (-2*(pf - p0)/(T**3)) + (v0 + vf)/(T**2)

    def cubic_pos(t, joint):
        tlim = min(max(t, 0.0), duration)
        return a0[joint] + a1[joint]*tlim + a2[joint]*tlim**2 + a3[joint]*tlim**3

    while True:
        t = time.perf_counter() - start_time
        if t > duration:
            break
        pos = {j: cubic_pos(t, j) for j in desired_position}
        bus.sync_write("Goal_Position", offset_config(pos), normalize=True)
        time.sleep(0.02)

    # Final correction
    bus.sync_write("Goal_Position", offset_config(desired_position), normalize=True)

    # Compute end velocities (to continue smoothly)
    v_end = {j: v0_dict[j] + 2*a2[j]*duration + 3*a3[j]*duration**2 for j in desired_position}
    return v_end
