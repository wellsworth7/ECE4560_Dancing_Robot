# import numpy as np
# import matplotlib
# matplotlib.use("Agg")  # Force non-GUI backend
# import matplotlib.pyplot as plt
# from pathlib import Path

# # --- CONFIG ---
# NPZ_FILE = Path(r"e:\Documents\Meera_NUS_acad\Y3-S1 Gt\ECE4560\ECE4560_Dancing_Robot\so101_kinematics\plots\joint_log_1763586842.npz")

# OUTPUT_DIR = NPZ_FILE.parent / "imgs"
# OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# # --- PLOT FUNCTION (now uses timestamps) ---
# def plot_joint_data(joint_name, timestamps, data, data_type, output_dir):
#     plt.figure(figsize=(8, 4), dpi=200)

#     plt.plot(timestamps, data, linewidth=1.5)
#     plt.title(f"{joint_name} {data_type}", fontsize=12)
#     plt.xlabel("Time [s]")
#     plt.ylabel(data_type)
#     plt.grid(True, alpha=0.3)
#     plt.tight_layout()

#     # sanitize filenames
#     safe_name = f"{joint_name}_{data_type}".replace(" ", "_").replace("/", "_")
#     plt.savefig(output_dir / f"{safe_name}.png", bbox_inches="tight")
#     plt.close()

# # --- LOAD DATA ---
# data = np.load(NPZ_FILE)

# pos = data["pos"]      # shape (N, 6)
# vel = data["vel"]      # shape (N, 6)
# acc = data["acc"]      # shape (N, 6)
# timestamps = data["timestamps"]  # shape (N,)
# pos = pos.T
# vel = vel.T
# acc = acc.T

# joint_names = ["shoulder_pan", "shoulder_lift", "elbow",
#                "wrist_1", "wrist_2", "wrist_3"]
# print("pos shape:", pos.shape)
# print("vel shape:", vel.shape)
# print("acc shape:", acc.shape)
# print("timestamps shape:", timestamps.shape)

# # --- CHECK SHAPES ---
# assert pos.shape[0] == timestamps.shape[0], "timestamps length does not match pos data length!"

# # # --- PLOT ALL JOINTS ---
# # for i, joint in enumerate(joint_names):
# #     plot_joint_data(joint, timestamps, pos[:, i], "position_deg", OUTPUT_DIR)
# #     plot_joint_data(joint, timestamps, vel[:, i], "velocity_deg_s", OUTPUT_DIR)
# #     plot_joint_data(joint, timestamps, acc[:, i], "acceleration_deg_s2", OUTPUT_DIR)

# # Relative timestamps
# timestamps_rel = timestamps - timestamps[0]

# print("First 10 relative timestamps:", timestamps_rel[:10])
# print("Min:", timestamps_rel.min(), "Max:", timestamps_rel.max())
# # Mask for 25–30 seconds
# mask = (timestamps_rel >= 10) & (timestamps_rel <= 20)
# timestamps_window = timestamps_rel[mask]


# # --- PLOT ONLY TIMESTAMPS > 1000 ---
# for i, joint in enumerate(joint_names):
#     plot_joint_data(joint, timestamps_window, pos[mask, i], "position_deg", OUTPUT_DIR)
#     plot_joint_data(joint, timestamps_window, vel[mask, i], "velocity_deg_s", OUTPUT_DIR)
#     plot_joint_data(joint, timestamps_window, acc[mask, i], "acceleration_deg_s2", OUTPUT_DIR)

# print(f"Plots for timestamps > 1000 saved in {OUTPUT_DIR}")

import numpy as np
import matplotlib
matplotlib.use("Agg")  # Non-GUI backend
import matplotlib.pyplot as plt
from pathlib import Path

# --- CONFIG ---
NPZ_FILE = Path(r"e:\Documents\Meera_NUS_acad\Y3-S1 Gt\ECE4560\ECE4560_Dancing_Robot\so101_kinematics\plots\joint_log_1763586842.npz")
OUTPUT_DIR = NPZ_FILE.parent / "imgs"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# --- PLOT FUNCTION ---
def plot_joint_data(joint_name, timestamps_sec, data, data_type, output_dir):
    plt.figure(figsize=(8, 4), dpi=200)
    plt.plot(timestamps_sec, data, linewidth=1.5)
    plt.title(f"{joint_name} {data_type}", fontsize=12)
    plt.xlabel("Time [s]")  # seconds
    plt.ylabel(data_type)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    safe_name = f"{joint_name}_{data_type}".replace(" ", "_").replace("/", "_")
    plt.savefig(output_dir / f"{safe_name}.png", bbox_inches="tight")
    plt.close()

# --- LOAD DATA ---
data = np.load(NPZ_FILE)
pos = data["pos"].T  # transpose so shape = (time, joints)
vel = data["vel"].T
acc = data["acc"].T
timestamps = data["timestamps"]

joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]
# --- RELATIVE TIMESTAMPS IN SECONDS ---
timestamps_rel = timestamps - timestamps[0]
print("First 10 relative timestamps [s]:", timestamps_rel[:10])
print("Max relative time [s]:", timestamps_rel.max())

# --- MASK FOR TIME WINDOW (e.g., 10–20s) ---
mask = (timestamps_rel >= 10) & (timestamps_rel <= 20)
timestamps_window = timestamps_rel[mask]

# --- PLOT FOR SELECTED WINDOW ---
for i, joint in enumerate(joint_names):
    plot_joint_data(joint, timestamps_window, pos[mask, i], "position_deg", OUTPUT_DIR)
    plot_joint_data(joint, timestamps_window, vel[mask, i], "velocity_deg_s", OUTPUT_DIR)
    plot_joint_data(joint, timestamps_window, acc[mask, i], "acceleration_deg_s2", OUTPUT_DIR)

print(f"Plots for timestamps 10–20s saved in {OUTPUT_DIR}")
