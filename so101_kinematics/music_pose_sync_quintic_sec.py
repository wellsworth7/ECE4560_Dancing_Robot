"""
Music-driven robot motion controller.
Each movement segment is timed with musical beats and shaped by features like energy and tempo.
This version expects POSE_FILE to be a JSON mapping:
{
  "intro": [ { "name": "...", "shoulder_pan": 0.0, ... }, ... ],
  "verse": [ ... ],
  ...
}
and SECTION_FILE to contain segments with "start","end","label".
"""
import json
import time
import numpy as np
import pygame
from pathlib import Path
from so101_utils import *   # assume this provides motor helpers & perform_quintic_move_smooth, offset_config, etc.

# ============================================================
# CONFIGURATION
# ============================================================
PORT_ID = "COM21"
ROBOT_NAME = "follower_arm"

DESIRED_MOVE_TIME = 1.5   # desired duration (s) between motions
ANTICIPATION = 0.5        # start moving before beat (s)
HOLD_MARGIN = 1.0         # time to hold pose before next move

POSE_FILE = Path(__file__).parent / "pose_sec_dictionary.json"
BEAT_FILE = Path(__file__).parent.parent / "music_processing/we_will_rock_you_features.json"
MUSIC_FILE = Path(__file__).parent.parent / "music_processing/we_will_rock_you.mp3"
SECTION_FILE = Path(__file__).parent.parent / "music_processing/we_will_rock_you.json"

# Joints where we must enforce limits (min,max)
JOINT_LIMITS = {
    "shoulder_pan": (-90, 90),
    "wrist_roll": (-165, 165),
    "gripper": (2, 95)
}

# Per-section scaling rules (used by map_music_to_pose_features)
SECTION_RULES = {
    "intro": {"shoulder_amplitude": 0.3, "wrist_amplitude": 1.0, "gripper_fraction": 0.2},
    "verse": {"shoulder_amplitude": 0.5, "wrist_amplitude": 1.0, "gripper_fraction": 0.0},
    "chorus": {"shoulder_amplitude": 1.0, "wrist_amplitude": 1.0, "gripper_fraction": 0.0},
    "instr": {"shoulder_amplitude": 1.0, "wrist_amplitude": 1.0, "gripper_fraction": 0.0},
    "end": {"shoulder_amplitude": 0.2, "wrist_amplitude": 1.0, "gripper_fraction": 0.3}
}

# ============================================================
# UTILITIES
# ============================================================
def load_sections(section_file=SECTION_FILE):
    with open(section_file, "r") as f:
        data = json.load(f)
    return data.get("segments", [])

def get_section_for_time(sections, t):
    for sec in sections:
        if sec["start"] <= t < sec["end"]:
            return sec["label"]
    return "verse"

def scale_to_joint(val, min_val, max_val):
    """Normalize [0,1] -> joint range (linear)."""
    return min_val + val * (max_val - min_val)

def load_beats_and_poses():
    """Load beat-feature list and pose dictionary (section -> [pose objects])."""
    with open(BEAT_FILE, "r") as f:
        beats = [b for b in json.load(f) if b.get("beat")]
    with open(POSE_FILE, "r") as f:
        pose_dict = json.load(f)
    return beats, pose_dict

def detect_tempo(beats):
    if len(beats) < 2:
        return 1, 0.0, 0.0
    intervals = np.diff([b["time"] for b in beats])
    avg_interval = np.mean(intervals)
    tempo_bpm = 60.0 / avg_interval
    N = max(1, round(DESIRED_MOVE_TIME / avg_interval))
    print(f"Detected tempo ≈ {tempo_bpm:.1f} BPM → moving every N={N} beats (avg interval {avg_interval:.3f}s)")
    return N, avg_interval, tempo_bpm

def init_music():
    pygame.mixer.init()
    pygame.mixer.music.load(MUSIC_FILE)

def setup_robot():
    calibration = load_calibration(ROBOT_NAME)
    bus = setup_motors(calibration, PORT_ID, gains={"P": 10, "I": 5, "D": 4})
    start_pose = bus.sync_read("Present_Position")
    print("Recorded actual starting pose:", start_pose)
    return bus, start_pose

# ============================================================
# MAPPING MUSIC TO POSES
# ============================================================
def map_music_to_pose_features(base_pose, beat, sign_toggle, rules):
    """
    base_pose: dict of joint -> value (may include joints not in JOINT_LIMITS)
    beat: beat feature dict (contains energy, tempo, ...)
    rules: section scaling rules
    returns (pose_dict, energy, tempo) where pose_dict contains the same keys as base_pose but with
    shoulder_pan, wrist_roll, gripper modified according to music+rules.
    """
    pose = base_pose.copy()
    energy = min(1.0, float(beat.get("energy", 0.0)))
    tempo = float(beat.get("tempo", 0.0))

    # shoulder_pan from energy
    if "shoulder_pan" in pose:
        min_pan, max_pan = JOINT_LIMITS["shoulder_pan"]
        base = base_pose["shoulder_pan"]
        max_delta = (max_pan - min_pan) / 2.0   # half-range
        raw = base + energy * rules["shoulder_amplitude"] * max_delta * sign_toggle
        pose["shoulder_pan"] = float(np.clip(raw, min_pan, max_pan))

    # wrist_roll from tempo
    if "wrist_roll" in pose:
        min_roll, max_roll = JOINT_LIMITS.get("wrist_roll", (-180.0, 180.0))
        min_tempo, max_tempo = 60.0, 180.0
        tempo_norm = np.clip((tempo - min_tempo) / (max_tempo - min_tempo), 0.0, 1.0)
        base = base_pose["wrist_roll"]
        max_delta = (max_roll - min_roll) / 2.0 
        raw = base + energy * rules["wrist_amplitude"] * max_delta * sign_toggle
        # raw = scale_to_joint(tempo_norm * rules.get("wrist_amplitude", 1.0), min_roll, max_roll) * sign_toggle
        pose["wrist_roll"] = float(np.clip(raw, min_roll, max_roll))

    # gripper from section fraction
    if "gripper" in pose:
        min_g, max_g = JOINT_LIMITS.get("gripper", (0.0, 100.0))
        raw = min_g + (max_g - min_g) * rules.get("gripper_fraction", 0.0)
        pose["gripper"] = float(np.clip(raw, min_g, max_g))

    # other joints stay as in base_pose
    return pose, energy, tempo

def add_midpoint_waypoint(current_pose, target_pose, sign_toggle, roll_amplitude=0.8, grip_open_frac=0.6):
    midpoint = {}
    for joint in current_pose:
        if joint == "wrist_roll":
            min_r, max_r = JOINT_LIMITS.get("wrist_roll", (-180.0, 180.0))
            center = 0.5 * (min_r + max_r)
            range_r = (max_r - min_r) / 2.0
            overshoot = center + sign_toggle * range_r * roll_amplitude
            midpoint[joint] = float(np.clip(overshoot, min_r, max_r))
        elif joint == "gripper":
            min_g, max_g = JOINT_LIMITS.get("gripper", (0.0, 100.0))
            midpoint[joint] = float(min_g + (max_g - min_g) * grip_open_frac)
        else:
            # If joints missing in target_pose, use current average
            if joint in target_pose:
                midpoint[joint] = 0.5 * (current_pose[joint] + target_pose[joint])
            else:
                midpoint[joint] = current_pose[joint]
    return midpoint

# ============================================================
# MAIN PERFORMANCE LOOP
# ============================================================
def perform_motion_sequence(bus, beats, pose_dict, start_pose):
    """
    pose_dict is expected to be { section_label: [ pose_obj, ... ], ... }
    where pose_obj is a dict containing joint names and numeric values. Optionally a "name" field.
    """
    # Per-section index map so each section keeps its own cycling index
    section_indices = {sec: 0 for sec in pose_dict.keys()}
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

    sections = load_sections(SECTION_FILE)

    for i in range(len(beats) - 1):
        if i % N == (N - 1):
            beat = beats[i]
            current_time = beat["time"]

            # compute move timing
            move_start = max(0.0, current_time - ANTICIPATION)
            next_move_time = beats[i + N]["time"] if i + N < len(beats) else beats[-1]["time"] + 1.0
            move_duration = max(0.1, next_move_time - move_start - HOLD_MARGIN)

            # determine current section label and rules
            section_label = get_section_for_time(sections, current_time)
            rules = SECTION_RULES.get(section_label, SECTION_RULES.get("verse", {}))

            # pick the list of poses for this section; fallback to concatenation of all sections
            available = pose_dict.get(section_label)
            if not available:
                # fallback: flatten all section lists into one
                available = []
                for lst in pose_dict.values():
                    available.extend(lst)

            if not available:
                print(f"[WARN] No poses defined anywhere — skipping beat {i}")
                continue

            # pick current index for this section and advance only within that section
            idx = section_indices.get(section_label, 0)
            pose_obj = available[idx]
            # pose_obj may include a "name" field; remove for base_pose if present
            base_pose = {k: float(v) for k, v in pose_obj.items() if k != "name"}

            # generate dynamic target pose from base and music features
            target_pose, energy, tempo = map_music_to_pose_features(base_pose, beat, sign_toggle, rules)

            # safety clipping for joints that have limits (extra safeguard)
            for j, val in list(target_pose.items()):
                if j in JOINT_LIMITS:
                    mn, mx = JOINT_LIMITS[j]
                    if val < mn or val > mx:
                        print(f"[CLIP] joint '{j}' value {val:.3f} out of [{mn},{mx}] -> clipped")
                        target_pose[j] = float(np.clip(val, mn, mx))

            # increment index for that section
            section_indices[section_label] = (idx + 1) % len(available)
            sp = target_pose.get("shoulder_pan", None)
            wr = target_pose.get("wrist_roll", None)
            # print status
            pose_name = pose_obj.get("name", f"{section_label}[{idx}]")
            print(
            f"[t={move_start:.3f}s] Beat #{i} → moving to {pose_name} | "
            f"section={section_label} | energy={energy:.3f}, tempo={tempo:.1f}, "
            f"shoulder_pan={sp}, wrist_roll={wr}"
            )

            # wait until move_start (synchronized to music playback)
            now = time.perf_counter() - start_perf + first_beat_time
            time.sleep(max(0.0, move_start - now))

            # read current pose
            current_pose = bus.sync_read("Present_Position")

            # optionally compute a midpoint (not used by the quintic in so101_utils unless you modify it)
            mid_pose = add_midpoint_waypoint(current_pose, target_pose, sign_toggle)

            # call motion executor (your perform_quintic_move_smooth should accept these joints)
            v_prev = perform_quintic_move_smooth(bus, current_pose, target_pose, move_duration, v_prev, sign_toggle=sign_toggle)

            # alternate sign for next cycle
            sign_toggle *= -1

        # align with next beat (sleep until next beat)
        now = time.perf_counter() - start_perf + first_beat_time
        next_beat_time = beats[i + 1]["time"]
        time.sleep(max(0.0, next_beat_time - now))

    # wrap up
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
        try:
            pygame.mixer.music.stop()
        except Exception:
            pass
        try:
            bus.disable_torque()
        except Exception:
            pass
        print("Motors disabled.")
