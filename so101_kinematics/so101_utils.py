# so101-utils.py
import numpy as np
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


def setup_motors(calibration, PORT_ID, gains=None):
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
    if gains is None:
        gains = {"P": 16, "I": 0, "D": 32}
    with bus.torque_disabled():
        bus.configure_motors()
        for name, motor in bus.motors.items():
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            bus.write("P_Coefficient", name, gains["P"])
            bus.write("I_Coefficient", name, gains["I"])
            bus.write("D_Coefficient", name, gains["D"])
            print(f"[INFO] {name} gains set: P={gains['P']} I={gains['I']} D={gains['D']}")    
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

def move_to_pose_quintic(bus, start_position, desired_position, duration,
                        v0_dict=None, v1_dict=None, a0_dict=None, a1_dict=None):
    """
    Smooth quintic trajectory interpolation (C2 continuity).
    """
    start_time = time.perf_counter()
    v0_dict = v0_dict or {j: 0.0 for j in desired_position}
    v1_dict = v1_dict or {j: 0.0 for j in desired_position}
    a0_dict = a0_dict or {j: 0.0 for j in desired_position}
    a1_dict = a1_dict or {j: 0.0 for j in desired_position}

    a0, a1, a2, a3, a4, a5 = {}, {}, {}, {}, {}, {}
    T = duration

    for j in desired_position:
        q0 = start_position[j]
        qf = desired_position[j]
        v0 = v0_dict[j]
        vf = v1_dict[j]
        acc0 = a0_dict[j]
        accf = a1_dict[j]

        a0[j] = q0
        a1[j] = v0
        a2[j] = acc0 / 2
        a3[j] = (20*(qf - q0) - (8*vf + 12*v0)*T - (3*acc0 - accf)*(T**2)) / (2*T**3)
        a4[j] = (30*(q0 - qf) + (14*vf + 16*v0)*T + (3*acc0 - 2*accf)*(T**2)) / (2*T**4)
        a5[j] = (12*(qf - q0) - (6*vf + 6*v0)*T - (acc0 - accf)*(T**2)) / (2*T**5)

    def pos(t, j):
        t = np.clip(t, 0, T)
        return (a0[j] + a1[j]*t + a2[j]*t**2 + a3[j]*t**3 + a4[j]*t**4 + a5[j]*t**5)

    while True:
        t = time.perf_counter() - start_time
        if t > T:
            break
        pos_dict = {j: pos(t, j) for j in desired_position}
        bus.sync_write("Goal_Position", offset_config(pos_dict), normalize=True)
        time.sleep(0.02)

    bus.sync_write("Goal_Position", offset_config(desired_position), normalize=True)
    v_end = {j: v0_dict[j] + 2*a2[j]*T + 3*a3[j]*T**2 + 4*a4[j]*T**3 + 5*a5[j]*T**4 for j in desired_position}
    return v_end

def move_to_pose_quintic2(bus, start_position, desired_position, duration,
                        v0_dict=None, v1_dict=None, a0_dict=None, a1_dict=None,
                        active_joints=None):
    """
    Smooth quintic trajectory interpolation (C2 continuity).
    Only joints in active_joints will move; others remain at start_position.
    """
    start_time = time.perf_counter()
    v0_dict = v0_dict or {j: 0.0 for j in desired_position}
    v1_dict = v1_dict or {j: 0.0 for j in desired_position}
    a0_dict = a0_dict or {j: 0.0 for j in desired_position}
    a1_dict = a1_dict or {j: 0.0 for j in desired_position}

    active_joints = active_joints or list(desired_position.keys())

    a0, a1, a2, a3, a4, a5 = {}, {}, {}, {}, {}, {}
    T = duration

    for j in active_joints:
        q0 = start_position[j]
        qf = desired_position[j]
        v0 = v0_dict.get(j, 0.0)
        vf = v1_dict.get(j, 0.0)
        acc0 = a0_dict.get(j, 0.0)
        accf = a1_dict.get(j, 0.0)

        a0[j] = q0
        a1[j] = v0
        a2[j] = acc0 / 2
        a3[j] = (20*(qf - q0) - (8*vf + 12*v0)*T - (3*acc0 - accf)*(T**2)) / (2*T**3)
        a4[j] = (30*(q0 - qf) + (14*vf + 16*v0)*T + (3*acc0 - 2*accf)*(T**2)) / (2*T**4)
        a5[j] = (12*(qf - q0) - (6*vf + 6*v0)*T - (acc0 - accf)*(T**2)) / (2*T**5)

    def pos(t, j):
        t = np.clip(t, 0, T)
        return a0[j] + a1[j]*t + a2[j]*t**2 + a3[j]*t**3 + a4[j]*t**4 + a5[j]*t**5

    while True:
        t = time.perf_counter() - start_time
        if t > T:
            break
        pos_dict = start_position.copy()  # default: hold all joints
        for j in active_joints:
            pos_dict[j] = pos(t, j)
        bus.sync_write("Goal_Position", offset_config(pos_dict), normalize=True)
        time.sleep(0.02)

    # Ensure final position is exactly reached
    final_pos = start_position.copy()
    for j in active_joints:
        final_pos[j] = desired_position[j]
    bus.sync_write("Goal_Position", offset_config(final_pos), normalize=True)

    # Compute end velocities only for active joints
    v_end = {j: v0_dict.get(j,0.0) + 2*a2[j]*T + 3*a3[j]*T**2 + 4*a4[j]*T**3 + 5*a5[j]*T**4
             for j in active_joints}
    return v_end

def perform_quintic_move_smooth(bus, start_pose, target_pose, move_duration, v_prev, sign_toggle):
    """
    Smooth motion from start -> target using quintic trajectory.
    - Adds expressive midpoint bump for wrist_roll and gripper.
    - Computes end velocities for smooth continuity.
    """
    start_time = time.perf_counter()
    dt = 0.02  # control loop timestep

    # Precompute quintic coefficients for all joints (start -> target)
    a0, a1, a2, a3, a4, a5 = {}, {}, {}, {}, {}, {}
    for j in start_pose:
        q0 = start_pose[j]
        qf = target_pose[j]
        v0 = v_prev.get(j, 0.0)
        vf = 0.0  # final velocity zero
        acc0 = 0.0
        accf = 0.0

        T = move_duration
        a0[j] = q0
        a1[j] = v0
        a2[j] = acc0 / 2
        a3[j] = (20*(qf - q0) - (8*vf + 12*v0)*T - (3*acc0 - accf)*T**2) / (2*T**3)
        a4[j] = (30*(q0 - qf) + (14*vf + 16*v0)*T + (3*acc0 - 2*accf)*T**2) / (2*T**4)
        a5[j] = (12*(qf - q0) - (6*vf + 6*v0)*T - (acc0 - accf)*T**2) / (2*T**5)

    # Midpoint bump parameters for expressive joints
    bump_joints = ["wrist_roll", "gripper"]
    bump_amplitude = {
        "wrist_roll": 0.8*(JOINT_LIMITS["wrist_roll"][1]-JOINT_LIMITS["wrist_roll"][0])/2 * sign_toggle,
        "gripper": 0.6*(JOINT_LIMITS["gripper"][1]-JOINT_LIMITS["gripper"][0])
    }

    while True:
        t = time.perf_counter() - start_time
        if t > move_duration:
            break

        pos_dict = {}
        for j in start_pose:
            q = a0[j] + a1[j]*t + a2[j]*t**2 + a3[j]*t**3 + a4[j]*t**4 + a5[j]*t**5
            # Add midpoint bump for wrist/gripper
            if j in bump_joints:
                q += bump_amplitude[j] * np.sin(np.pi * t / move_duration)
            pos_dict[j] = q

        bus.sync_write("Goal_Position", offset_config(pos_dict), normalize=True)
        time.sleep(dt)

    # Ensure final position is exactly reached
    bus.sync_write("Goal_Position", offset_config(target_pose), normalize=True)

    # Compute end velocities (for continuity to next segment)
    v_end = {}
    for j in start_pose:
        T = move_duration
        v_end[j] = a1[j] + 2*a2[j]*T + 3*a3[j]*T**2 + 4*a4[j]*T**3 + 5*a5[j]*T**4
    return v_end
