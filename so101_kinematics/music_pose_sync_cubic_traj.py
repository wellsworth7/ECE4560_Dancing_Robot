import json
import time
import pygame
import numpy as np
from pathlib import Path
from so101_utils import *

# -----------------------------
# CONFIG
# -----------------------------
PORT_ID = "COM21"
ROBOT_NAME = "follower_arm"

DESIRED_MOVE_TIME = 1.5  # desired duration (s) between motions
ANTICIPATION = 0.5   # start moving this many seconds before the beat
HOLD_MARGIN = 1.0   # hold until 1s before next movement

POSE_FILE = Path(__file__).parent / "pose_dictionary.json"
BEAT_FILE = Path(__file__).parent.parent / "music_processing/music_features_tempo_sync.json"
MUSIC_FILE = Path(__file__).parent.parent / "music_processing/we_will_rock_you.mp3"

# -----------------------------
# LOAD DATA
# -----------------------------
with open(BEAT_FILE, "r") as f:
    beat_data = json.load(f)
beats = [b for b in beat_data if b["beat"]]

with open(POSE_FILE, "r") as f:
    pose_dict = json.load(f)

pose_names = list(pose_dict.keys())
pose_index = 0
gripper_state = False
sign_toggle = 1

# -----------------------------
# AUTO-DETECT N BASED ON TEMPO
# -----------------------------
if len(beats) > 1:
    beat_intervals = np.diff([b["time"] for b in beats])
    avg_interval = np.mean(beat_intervals)
    tempo_bpm = 60.0 / avg_interval

    # Compute N so each movement lasts around DESIRED_MOVE_TIME seconds
    N = max(1, round(DESIRED_MOVE_TIME / avg_interval))
else:
    N = 1
    avg_interval = 0
    tempo_bpm = 0

print(f"Detected tempo ≈ {tempo_bpm:.1f} BPM → moving every N={N} beats (avg interval {avg_interval:.3f}s)")

# -----------------------------
# INIT MUSIC
# -----------------------------
pygame.mixer.init()
pygame.mixer.music.load(MUSIC_FILE)

# -----------------------------
# SETUP ROBOT
# -----------------------------
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)

print("Recording starting pose...")
start_pose = bus.sync_read("Present_Position")
print("Recorded actual starting pose:", start_pose)

joint_limits = {
    "shoulder_pan": (-90, 90),
    "wrist_roll": (-165, 165),
    "gripper": (4, 95)
}

def scale_to_joint(val, min_val, max_val):
    return min_val + val * (max_val - min_val)


# -----------------------------
# MUSIC SYNC
# -----------------------------
print("Starting music...")
pygame.mixer.music.play()

first_beat_time = beats[0]["time"]
print(f"Waiting {first_beat_time:.3f}s until first beat...")
time.sleep(first_beat_time)

start_perf = time.perf_counter()
v_prev = {joint: 0.0 for joint in start_pose}
def add_midpoint_waypoint(current_pose, target_pose, joint_limits, sign_toggle, roll_amplitude=0.8, grip_open_frac=0.6):
    """
    Generate a midpoint waypoint:
      - Wrist roll overshoots its limit in one direction (then returns)
      - Gripper opens partially at midpoint (stays closed at poses)
      - Other joints move linearly between poses
    """
    midpoint_pose = {}

    for joint in current_pose:
        if joint == "wrist_roll":
            # Wrist roll overshoot
            min_roll, max_roll = joint_limits["wrist_roll"]
            roll_center = 0.5 * (min_roll + max_roll)
            roll_range = (max_roll - min_roll) / 2
            overshoot = roll_center + sign_toggle * roll_range * roll_amplitude
            midpoint_pose[joint] = np.clip(overshoot, min_roll, max_roll)

        elif joint == "gripper":
            # Gripper opens partially at midpoint
            min_g, max_g = joint_limits["gripper"]
            midpoint_pose[joint] = min_g + (max_g - min_g) * grip_open_frac

        else:
            # Normal linear midpoint for other joints
            midpoint_pose[joint] = 0.5 * (current_pose[joint] + target_pose[joint])

    return midpoint_pose



# -----------------------------
# MAIN LOOP
# -----------------------------
try:
    for i in range(len(beats) - 1):
        if i % N == (N - 1):
            move_start_time = max(0.0, beats[i]["time"] - ANTICIPATION)
            next_move_time = beats[i + N]["time"] if i + N < len(beats) else beats[-1]["time"] + 1.0
            move_duration = max(0.1, next_move_time - move_start_time- HOLD_MARGIN )

            target_pose = pose_dict[pose_names[pose_index]].copy()
            pose_index = (pose_index + 1) % len(pose_names)

            # Map energy → shoulder_pan (alternating sign)
            energy = min(1.0, beats[i]["energy"])
            min_pan, max_pan = joint_limits["shoulder_pan"]
            target_pose["shoulder_pan"] = scale_to_joint(energy, min_pan, max_pan) * sign_toggle
            #Map tempo → wrist_roll (with alternating sign)
            tempo = beats[i]["tempo"]
            min_tempo, max_tempo = 60, 180
            tempo_norm = min(max((tempo - min_tempo) / (max_tempo - min_tempo), 0), 1)
            min_roll, max_roll = joint_limits["wrist_roll"]
            wrist_roll_val = scale_to_joint(tempo_norm, min_roll, max_roll)
            target_pose["wrist_roll"] = wrist_roll_val * sign_toggle
            # Alternate gripper
            # gripper_state = not gripper_state
            # target_pose["gripper"] = joint_limits["gripper"][1] if gripper_state else joint_limits["gripper"][0]
            target_pose["gripper"] = joint_limits["gripper"][0]


            print(f"[t={move_start_time:.3f}s] Beat #{i} → START moving to {pose_names[pose_index]}"
                  f"(energy={energy:.3f}, shoulder_pan={target_pose['shoulder_pan']:.1f}, "
                  f"gripper={target_pose['gripper']}) "
                  f"wrist_roll={target_pose['wrist_roll']:.1f}, gripper={target_pose['gripper']})")

            # Wait until move start
            now = time.perf_counter() - start_perf + first_beat_time
            time.sleep(max(0.0, move_start_time - now))

            # Smooth motion
            current_pose = bus.sync_read("Present_Position")
            # --- compute midpoint as before
            mid_pose = add_midpoint_waypoint(current_pose, target_pose, joint_limits, sign_toggle)

            # durations
            half_duration = move_duration / 2.0

            # compute a continuous midpoint velocity (deg/s or units used by your motors)
            # v_const is the desired velocity at the internal boundary (midpoint)
            v_const = {}
            for j in target_pose:
                # avoid division by tiny duration
                v_const[j] = (target_pose[j] - current_pose[j]) / max(move_duration, 1e-6)

            # First segment: start from v_prev (incoming), end at v_const (no stop)
            v_mid = move_to_pose_cubic_cont(bus,
                                        start_position=current_pose,
                                        desired_position=mid_pose,
                                        duration=half_duration,
                                        v0_dict=v_prev,
                                        v1_dict=v_const)

            # Second segment: start at v_const, end continuing to whatever (use zeros or keep continuity by computing next v)
            v_end = move_to_pose_cubic_cont(bus,
                                            start_position=mid_pose,
                                            desired_position=target_pose,
                                            duration=half_duration,
                                            v0_dict=v_const,
                                            v1_dict={j: 0.0 for j in target_pose})  # final end velocity (can be changed)

            # keep v_prev for next move
            v_prev = v_end


            # Hold till next
            # hold_duration = max(0.0, next_move_time - HOLD_MARGIN -
            #                     (time.perf_counter() - start_perf + first_beat_time))
            # hold_position(bus, hold_duration)

            sign_toggle *= -1

        # Align with next beat
        now = time.perf_counter() - start_perf + first_beat_time
        next_beat_time = beats[i + 1]["time"]
        time.sleep(max(0.0, next_beat_time - now))

    # -----------------------------
    # RETURN TO START POSE
    # -----------------------------
    print("Song ending → returning to start pose...")
    move_to_pose_cubic_cont(bus, bus.sync_read("Present_Position"), start_pose, 1.5, v_prev)
    hold_position(bus, 0.5)

except KeyboardInterrupt:
    print("\nInterrupted by user (Ctrl+C).")

finally:
    print("Stopping music and disabling motor torque...")
    pygame.mixer.music.stop()
    bus.disable_torque()
    print("Motors disabled.")
