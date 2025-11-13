"""
Music-driven robot motion controller.
Each movement segment is timed with musical beats and shaped by features like energy and tempo.
"""

import json
import time
import numpy as np
import pygame
from pathlib import Path
from so101_utils import *

# ============================================================
# CONFIGURATION
# ============================================================

PORT_ID = "COM21"
ROBOT_NAME = "follower_arm"

DESIRED_MOVE_TIME = 1.5   # desired duration (s) between motions
ANTICIPATION = 0.5        # start moving before beat (s)
HOLD_MARGIN = 1.0        # time to hold pose before next move

POSE_FILE = Path(__file__).parent / "pose_dictionary.json"
BEAT_FILE = Path(__file__).parent.parent / "music_processing/we_will_rock_you_features.json"
MUSIC_FILE = Path(__file__).parent.parent / "music_processing/we_will_rock_you.mp3"
JOINT_LIMITS = {
    "shoulder_pan": (-90, 90),
    "wrist_roll": (-165, 165),
    "gripper": (4, 95)
}

# ============================================================
# UTILITY FUNCTIONS
# ============================================================

def scale_to_joint(val, min_val, max_val):
    """Normalize [0,1] → joint range."""
    return min_val + val * (max_val - min_val)

def load_beats_and_poses():
    """Load music beat features and pose dictionary."""
    with open(BEAT_FILE, "r") as f:
        beats = [b for b in json.load(f) if b["beat"]]
    with open(POSE_FILE, "r") as f:
        pose_dict = json.load(f)
    return beats, pose_dict

def detect_tempo(beats):
    """Estimate tempo (BPM) and number of beats per move."""
    if len(beats) < 2:
        return 1, 0.0, 0.0
    intervals = np.diff([b["time"] for b in beats])
    avg_interval = np.mean(intervals)
    tempo_bpm = 60.0 / avg_interval
    N = max(1, round(DESIRED_MOVE_TIME / avg_interval))
    print(f"Detected tempo ≈ {tempo_bpm:.1f} BPM → moving every N={N} beats (avg interval {avg_interval:.3f}s)")
    return N, avg_interval, tempo_bpm

def init_music():
    """Initialize audio playback."""
    pygame.mixer.init()
    pygame.mixer.music.load(MUSIC_FILE)

def setup_robot():
    """Initialize motors and read starting pose."""
    calibration = load_calibration(ROBOT_NAME)
    # bus = setup_motors(calibration, PORT_ID)
    bus = setup_motors(calibration, PORT_ID, gains={"P": 10, "I": 5, "D": 4})

    start_pose = bus.sync_read("Present_Position")
    print("Recorded actual starting pose:", start_pose)
    return bus, start_pose

# ============================================================
# MUSIC-FEATURE TO POSE MAPPING
# ============================================================

def map_music_to_pose_features(base_pose, beat, sign_toggle):
    """Map music features → target joint configuration."""
    pose = base_pose.copy()

    # --- Extract features
    energy = min(1.0, beat["energy"])
    tempo = beat["tempo"]

    # Shoulder_pan ← energy (alternating direction)
    min_pan, max_pan = JOINT_LIMITS["shoulder_pan"]
    pose["shoulder_pan"] = scale_to_joint(energy, min_pan, max_pan) * sign_toggle

    # Wrist_roll ← tempo (alternating direction)
    min_tempo, max_tempo = 60, 180
    tempo_norm = np.clip((tempo - min_tempo) / (max_tempo - min_tempo), 0, 1)
    min_roll, max_roll = JOINT_LIMITS["wrist_roll"]
    wrist_val = scale_to_joint(tempo_norm, min_roll, max_roll)
    pose["wrist_roll"] = wrist_val * sign_toggle

    # Gripper static (closed)
    pose["gripper"] = JOINT_LIMITS["gripper"][0]
    return pose, energy, tempo

def add_midpoint_waypoint(current_pose, target_pose, sign_toggle, roll_amplitude=0.8, grip_open_frac=0.6):
    """Generate an expressive midpoint waypoint."""
    midpoint = {}
    for joint in current_pose:
        if joint == "wrist_roll":
            min_r, max_r = JOINT_LIMITS["wrist_roll"]
            center = 0.5 * (min_r + max_r)
            range_r = (max_r - min_r) / 2
            overshoot = center + sign_toggle * range_r * roll_amplitude
            midpoint[joint] = np.clip(overshoot, min_r, max_r)
        elif joint == "gripper":
            min_g, max_g = JOINT_LIMITS["gripper"]
            midpoint[joint] = min_g + (max_g - min_g) * grip_open_frac
        else:
            midpoint[joint] = 0.5 * (current_pose[joint] + target_pose[joint])
    return midpoint

# ============================================================
# MAIN PERFORMANCE LOOP
# ============================================================

def perform_motion_sequence(bus, beats, pose_dict, start_pose):
    """Synchronize robot motion with music beats."""
    pose_names = list(pose_dict.keys())
    pose_index = 0
    sign_toggle = 1
    v_prev = {joint: 0.0 for joint in start_pose}

    N, avg_interval, tempo_bpm = detect_tempo(beats)
    init_music()

    # Wait until first beat
    first_beat_time = beats[0]["time"]
    pygame.mixer.music.play()
    print(f"Waiting {first_beat_time:.3f}s until first beat...")
    time.sleep(first_beat_time)
    start_perf = time.perf_counter()

    for i in range(len(beats) - 1):
        if i % N == (N - 1):
            move_start = max(0.0, beats[i]["time"] - ANTICIPATION)
            next_move_time = beats[i + N]["time"] if i + N < len(beats) else beats[-1]["time"] + 1.0
            move_duration = max(0.1, next_move_time - move_start - HOLD_MARGIN)

            # --- Generate target pose
            base_pose = pose_dict[pose_names[pose_index]]
            target_pose, energy, tempo = map_music_to_pose_features(base_pose, beats[i], sign_toggle)
            pose_index = (pose_index + 1) % len(pose_names)

            print(f"[t={move_start:.3f}s] Beat #{i} → moving to {pose_names[pose_index]}"
                  f" | energy={energy:.3f}, tempo={tempo:.1f}")

            # Wait for sync
            now = time.perf_counter() - start_perf + first_beat_time
            time.sleep(max(0.0, move_start - now))

            # Current pose & midpoint
            current_pose = bus.sync_read("Present_Position")
            mid_pose = add_midpoint_waypoint(current_pose, target_pose, sign_toggle)

            # Execute smooth motion
            v_prev = perform_quintic_move_smooth(bus, current_pose, target_pose, move_duration, v_prev, sign_toggle=sign_toggle)

            # Alternate direction next cycle
            sign_toggle *= -1

        # Align with next beat
        now = time.perf_counter() - start_perf + first_beat_time
        next_beat_time = beats[i + 1]["time"]
        time.sleep(max(0.0, next_beat_time - now))

    # Wrap-up motion
    print("Song ending → returning to start pose...")
    move_to_pose_cubic_cont(bus, bus.sync_read("Present_Position"), start_pose, 1.5, v_prev)
    hold_position(bus, 0.5)

# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    try:
        beats, pose_dict = load_beats_and_poses()
        bus, start_pose = setup_robot()
        perform_motion_sequence(bus, beats, pose_dict, start_pose)

    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C).")

    finally:
        print("Stopping music and disabling motor torque...")
        pygame.mixer.music.stop()
        bus.disable_torque()
        print("Motors disabled.")
