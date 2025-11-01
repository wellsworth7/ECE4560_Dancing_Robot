import math
import numpy as np
from so101_forward_kinematics import *

L2 = 0.116   # joint2 to joint3
L3 = 0.1349  # joint3 to wrist (joint4)
BASE_OFFSET_X = 0.0388353
BASE_OFFSET_DELTA = 0.0303992
Z_OFFSETS = 0.0542 + 0.0624
PHI = math.atan2(0.028, 0.11257)

def get_inverse_kinematics(target_position, target_orientation):
    "Geometric approach specific to the so-101 arms"
    base_offset = np.array([BASE_OFFSET_X, 0.0, 0.0])
    x_des, y_des, z_des = np.array(target_position) - base_offset

    theta1 = math.degrees(math.atan2(-y_des, x_des))
    wrist_pos, wrist_ori = get_wrist_flex_position(target_position)
    print("Wrist position: ",wrist_pos)
    try:
        geom = compute_wrist_geometry(wrist_pos)
        theta2, theta3 = solve_theta2_theta3(geom)
        theta4 = solve_theta4(geom)
    except ValueError as e:
        print("Target out of reach:", e)
        return None
    theta5 = theta1
    
    # Initialize the joint configuration dictionary
    joint_config = {
        'shoulder_pan': theta1,
        'shoulder_lift': theta2,
        'elbow_flex': theta3,
        'wrist_flex': theta4,
        'wrist_roll': theta5,
        'gripper': 0.0
    }
    return joint_config

def get_wrist_flex_position(target_position):
    gwt = np.eye(4)
    gwt[0:3, 3] = target_position
    g4t = get_g45(0) @ get_g5t()
    gw4 = gwt @ np.linalg.inv(g4t)
    wrist_flex_position = gw4[0:3, 3]
    wrist_flex_orientation = gw4[0:3, 0:3]
    return wrist_flex_position, wrist_flex_orientation


def compute_wrist_geometry(wrist_pos):
    """
    Compute all repeated geometric quantities for wrist_pos.
    Returns a dict with delta_x, delta_z, D, alpha, beta, gamma, phi
    """
    delta_x = math.sqrt((wrist_pos[0]-BASE_OFFSET_X)**2 + wrist_pos[1]**2) - BASE_OFFSET_DELTA
    delta_z = wrist_pos[2] - Z_OFFSETS
    D = math.sqrt(delta_x**2 + delta_z**2)

    # if D > (L2 + L3):
    #     raise ValueError("Target out of reach: D > L2 + L3")

    cos_beta = (L2**2 + L3**2 - D**2) / (2 * L2 * L3)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = math.acos(cos_beta)

    cos_alpha = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = math.acos(cos_alpha)

    gamma = math.atan2(delta_z, delta_x)

    return {
        'delta_x': delta_x,
        'delta_z': delta_z,
        'D': D,
        'alpha': alpha,
        'beta': beta,
        'gamma': gamma,
        'phi': PHI
    }

def solve_theta2_theta3(geom):
    theta3 = np.pi/2 - geom['beta'] + geom['phi']
    theta2 = np.pi/2 - geom['gamma'] - geom['alpha'] - geom['phi']

    theta2_deg = math.degrees(theta2)
    theta3_deg = math.degrees(theta3)
    return theta2_deg, theta3_deg


def solve_theta4(geom):
    theta4 = geom['gamma'] + geom['alpha'] + geom['beta'] - np.pi/2
    theta4_deg = math.degrees(theta4)
  
    return theta4_deg