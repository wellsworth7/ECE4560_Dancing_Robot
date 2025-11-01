# Getting Started with LeRobot

If you're setting up LeRobot for the first time, follow these steps to get your environment ready.

## Quick Start
1. Install Miniforge (recommended for cross-platform compatibility)
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh


2. After installation, restart your shell and verify:

conda --version

3. Create and activate the environment
conda create -y -n lerobot python=3.10
conda activate lerobot

4. Install dependencies

Install ffmpeg (required by LeRobot):
conda install ffmpeg -c conda-forge

Then install Python dependencies:
pip install -r requirements.txt

# Calibrate the Robot

Finding the port the robot is connected to:
lerobot-find-port

To perform motor calibration using LeRobot, run the following command in your activated environment:

lerobot-calibrate --robot.type=so101_follower --robot.port={PORT_NAME} --robot.id=follower_arm --robot.calibration_dir="EC4560_Dancing_Robot/so101_kinematics/calibration_files"

Replace {PORT_NAME} with the port the arm is connected to.
