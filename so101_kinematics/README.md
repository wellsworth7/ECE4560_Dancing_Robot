# Getting Started with LeRobot

If you're setting up **LeRobot** for the first time, follow these steps to get your environment ready.

## Quick Start

### 1. Install Miniforge (recommended for cross-platform compatibility)

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

### 2. After installation, restart your shell and verify installation

```bash
conda --version
```

### 3. Create and activate the environment

```bash
conda create -y -n lerobot python=3.10
conda activate lerobot
```

### 4. Install dependencies

Install **ffmpeg** (required by LeRobot):

```bash
conda install ffmpeg -c conda-forge
```

Then install Python dependencies:

```bash
pip install -r requirements.txt
```
---

## Calibrate the Robot

### 1. Find the port the robot is connected to

```bash
lerobot-find-port
```

### 2. Perform motor calibration

Run the following command inside your activated environment:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port={PORT_NAME} --robot.id=follower_arm --robot.calibration_dir="EC4560_Dancing_Robot/so101_kinematics/calibration_files"
```

Replace `{PORT_NAME}` with the actual port your arm is connected to.
---

Once calibration is complete, a calibration file will be saved automatically to:

```
EC4560_Dancing_Robot/so101_kinematics/calibration_files/so101_follower.json
```

You can now use this calibration file when initializing your robot.
