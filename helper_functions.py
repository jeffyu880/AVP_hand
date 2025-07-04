from copy import deepcopy as cp
from scipy.spatial.transform import Rotation as R
import numpy as np

def get_relative_quaternion(q1, q2):
    r1 = R.from_quat(q1)
    r2 = R.from_quat(q2)
            
    return r1.inv() * r2

def disect_avp_message(data, latest, frame_name):
    rot_matrix = data[:3,:3]
    trans = data[:3, 3]

    quat = R.as_quat(R.from_matrix(rot_matrix))

    latest[frame_name] = [trans, quat]

    return latest

def get_single_finger_joint(latest:dict, finger_name):
    base_quat = latest[finger_name + "_Base_r"][1]
    mcp_quat = latest[finger_name + "_MCP_r"][1]
    pip_quat = latest[finger_name + "_PIP_r"][1]
    dip_quat = latest[finger_name + "_DIP_r"][1]

    diff_mcp = get_relative_quaternion(base_quat, mcp_quat).as_euler("zyx", degrees = True)
    diff_pip = get_relative_quaternion(mcp_quat, pip_quat).as_euler("zyx", degrees = True)
    diff_dip = get_relative_quaternion(pip_quat, dip_quat).as_euler("zyx", degrees = True)

    return {finger_name + "_MCP_Spread":diff_mcp[1], 
            finger_name + "_MCP": diff_mcp[0], 
            finger_name + "_PIP": diff_pip[0], 
            finger_name + "_DIP": diff_dip[0]}

def get_finger_joints(latest:dict):
    finger_joints = {}
    for finger_name in ["Index", "Middle", "Ring", "Pinky"]:
        finger_joints = cp(finger_joints | get_single_finger_joint(latest, finger_name))
        
    return finger_joints

def get_thumb_joints(latest:dict):
    thumb_joints = {}

    # Compute CMC stuff
    thumb_measurement = latest["Thumb_MCP_r"][0] - latest["Thumb_CMC_r"][0]
    thumb_measurement[2] -= 0.01 # Correction factor based on kinematic location ... need to look this up at some point

    hand_rotate = R.from_euler("z", -15, degrees=True)
    hand_rotate_m = hand_rotate.as_matrix()

    cmc1_rot = R.from_euler("y", 10.5, degrees=True)
    cmc1_rot_m = cmc1_rot.as_matrix()

    # cmc1_aligned: measured point in the frame aligned to CMC1
    hand_aligned = np.matmul(thumb_measurement, hand_rotate_m)
    cmc1_aligned = np.matmul(hand_aligned, cmc1_rot_m)

    # project cmc1_aligned to y-z plane
    cmc1_projected = np.array([0, cmc1_aligned[1], cmc1_aligned[2]])

    # Get angle of CMC1: z as the 'x' axis and -y as the 'y' axis
    cmc1 = np.degrees(np.arctan2(-cmc1_projected[1], cmc1_projected[2]))

    cmc2_rot = R.from_euler("x", cmc1, degrees=True)
    cmc2_rot_m = cmc2_rot.as_matrix()

    cmc2_aligned = np.matmul(cmc1_aligned, cmc2_rot_m)

    # Get angle of CMC2: z as the 'x' axis and -x as the 'y' axis
    cmc2 = np.degrees(np.arctan2(-cmc2_aligned[0], cmc2_aligned[2]))

    # Get MCP / IP joints
    cmc_quat = latest["Thumb_CMC_r"][1]
    mcp_quat = latest["Thumb_MCP_r"][1]
    ip_quat = latest["Thumb_IP_r"][1]

    diff_mcp = get_relative_quaternion(cmc_quat, mcp_quat).as_euler("zyx", degrees = True)
    diff_ip = get_relative_quaternion(mcp_quat, ip_quat).as_euler("zyx", degrees = True)

    thumb_joints["Thumb_CMC1"] = cmc1 #- 10
    thumb_joints["Thumb_CMC2"] = cmc2
    thumb_joints["Thumb_MCP"] = diff_mcp[0]
    thumb_joints["Thumb_IP"] = diff_ip[0] + 15
    
    return thumb_joints

def get_hand_joints(latest:dict):
    finger_joints = get_finger_joints(latest)
    thumb_joints = get_thumb_joints(latest)
    
    return finger_joints | thumb_joints

