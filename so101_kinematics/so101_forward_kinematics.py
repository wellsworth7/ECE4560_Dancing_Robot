import time
import mujoco
import numpy as np

def Rx(thetadeg):
    thetarad = np.deg2rad(thetadeg)
    c = np.cos(thetarad)
    s = np.sin(thetarad)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def Ry(thetadeg):
    thetarad = np.deg2rad(thetadeg)
    c = np.cos(thetarad)
    s = np.sin(thetarad)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])
    
def Rz(thetadeg):
    thetarad = np.deg2rad(thetadeg)
    c = np.cos(thetarad)
    s = np.sin(thetarad)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])

def get_gw1(theta1_deg):
    displacement = (0.0388353, 0.0, 0.0624)
    rotation = Ry(180) @ Rz(theta1_deg)
    pose = np.block([[rotation, np.array(displacement).reshape(3,1)], [0, 0, 0, 1]])
    return pose

def get_g12(theta2_deg):
    displacement = (-0.0303992  , 0.0, -0.0542)
    rotation = Ry(-90) @ Rx(-90) @ Rz(theta2_deg)
    pose = np.block([[rotation, np.array(displacement).reshape(3,1)], [0, 0, 0, 1]])
    return pose

def get_g23(theta3_deg):
    displacement =(-0.11257, -0.028, 0.0)
    rotation = Rz(90)@ Rz(theta3_deg)
    pose = np.block([[rotation, np.array(displacement).reshape(3,1)], [0, 0, 0, 1]])
    return pose

def get_g34(theta4_deg):
    displacement = (-0.1349, 0.0, 0.0)
    rotation = Rz(-90) @ Rz(theta4_deg)
    pose = np.block([[rotation, np.array(displacement).reshape(3,1)], [0, 0, 0, 1]])
    return pose

def get_g45(theta5_deg):
    displacement = (0.0, -0.0611, 0.0)
    rotation = Rx(-90) @ Rz(180) @ Rz(theta5_deg)
    pose = np.block([[rotation, np.array(displacement).reshape(3,1)], [0, 0, 0, 1]])
    return pose

def get_g5t():
    # gripper -> tool/object frame 
    displacement = (0.0, 0.0, -0.1034)
    # rotation = Rx(90)@Rz(-90)
    rotation = np.eye(3) #changed - why??
    pose = np.block([[rotation, np.array(displacement).reshape(3,1)],[0, 0, 0, 1]])
    return pose



def get_forward_kinematics(position_dict):
    gw1 = get_gw1(position_dict['shoulder_pan'])
    g12 = get_g12(position_dict['shoulder_lift'])
    g23 = get_g23(position_dict['elbow_flex'])
    g34 = get_g34(position_dict['wrist_flex'])
    g45 = get_g45(position_dict['wrist_roll'])
    g5t = get_g5t()

    gwt = gw1 @ g12 @ g23 @ g34 @ g45 @ g5t
    position = gwt[0:3, 3]
    rotation = gwt[0:3, 0:3]
    return position, rotation

