# music_pose_generator.py
"""
Music-Synchronized Pose Generator for SO-101 Arm
------------------------------------------------
This script generates time-synchronized poses for the SO-101 robot arm
based on music features and predefined poses.

Inputs:
  1. Music features JSON: ECE4560_Dancing_Robot/music_processing/music_features_tempo_sync.json
  2. Pose dictionary JSON: ECE4560_Dancing_Robot/pose_dictionary.json

Behavior:
  - Obtain 1 randomized shuffle having all 5 poses from the dictionary.
  - Change pose every Nth beat, where:
        T_beat = 60 / BPM
        T_max = 3.0 seconds
        N = floor(T_max / T_beat)
  - Modify three joints dynamically based on music data:
        * wrist_roll → tempo-based angular velocity integration
        * shoulder_pan → maps to music energy
        * gripper → alternates open/close every Nth beat ("clapping" effect)
  - Output: A list or stream of pose configurations synchronized with the music.
"""

import json
import random
import math
import time


# --- File Paths ---
MUSIC_FEATURES_PATH = "ECE4560_Dancing_Robot/music_processing/music_features_tempo_sync.json"
POSE_DICTIONARY_PATH = "ECE4560_Dancing_Robot/pose_dictionary.json"


def load_json(path):
    """Helper to load a JSON file."""
    with open(path, "r") as f:
        return json.load(f)


def main():
    # --- Load Input Files ---
    music_data = load_json(MUSIC_FEATURES_PATH)
    pose_dict = load_json(POSE_DICTIONARY_PATH)

    # --- Extract music features ---
    # TODO: Get BPM, energy, and any other relevant features from the JSON
    BPM = None
    energy_values = []

    # --- Compute timing parameters ---
    # TODO: Compute Tbeat, Tmax, N
    Tbeat = None
    Tmax = 3.0
    N = None

    # --- Shuffle poses ---
    pose_names = list(pose_dict.keys())
    random.shuffle(pose_names)

    # --- Initialize loop variables ---
    current_pose_index = 0
    wrist_angle = 0.0
    gripper_open = True

    # --- Main loop over beats or time ---
    # TODO: Iterate through beats or time steps, update joint values
    # Change pose every Nth beat
    # Map BPM → wrist angular velocity
    # Map energy → shoulder_pan
    # Alternate gripper open/close
    # Output pose

    # Example placeholder loop
    for step in range(10):
        # TODO: Fill in actual computation here
        pose_name = pose_names[current_pose_index]
        base_pose = pose_dict[pose_name].copy()

        # Modify wrist_roll, shoulder_pan, gripper joints here
        # base_pose["wrist_roll"] = ...
        # base_pose["shoulder_pan"] = ...
        # base_pose["gripper"] = ...

        # Output or send to robot
        print(f"Step {step}: Pose = {pose_name}, Pose Data = {base_pose}")

        time.sleep(T_beat) #Send at beat intervals


if __name__ == "__main__":
    main()