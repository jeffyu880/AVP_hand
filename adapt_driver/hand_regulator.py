import os
import json
import sys
import numpy as np
from copy import deepcopy as cp
from sensor_msgs.msg import JointState
import scipy.optimize
from .motor_regulator import MotorRegulator
from .utility import get_robot_id

class HandRegulator:

    def __init__(self) -> None:        
        # Get configuration ID
        id = get_robot_id("../robot_config")

        dir_path =  os.path.dirname(os.path.realpath(__file__))
        
        json_file = dir_path + "/../robot_config/" + id + "/hand.config.json"
        self.hand_config = json.load(open(json_file))

        json_file = dir_path + "/../robot_config/" + id + "/motor.config.json"
        self.motor_config = json.load(open(json_file))

        self.motor_regulator = MotorRegulator()

        # Magical angle correction ratio for spread joints
        self.spread_angle_correction = -1/200

    def _flex_dt_given_theta(self, theta, joint_length, tendon_lever, length_wo_t):
        if theta == 0:
            tendon_position = 0
        else:
            base_t = joint_length + 2 * length_wo_t
            current_t = 2 * (joint_length / theta - tendon_lever) * np.sin(theta / 2) + 2 * length_wo_t * np.cos(theta / 2)
            tendon_position = base_t - current_t

        return tendon_position
    
    def _theta_given_flex_dt(self, tendon_position, joint_length, tendon_lever, length_wo_t):

        def f(theta, joint_length, tendon_lever, length_wo_t):
            return self._flex_dt_given_theta(theta, joint_length, tendon_lever, length_wo_t) - tendon_position
        
        is_success = False
        for i in range(5):
            rand = 2 * cp(np.random.random()) -1
            scale = 1 + rand * 0.2
            limits = [(-np.pi/3)*scale, (np.pi/0.8)*scale]
            try:
                solution = scipy.optimize.root_scalar(f, args=(joint_length, tendon_lever, length_wo_t),
                                                    bracket = [-np.pi/3, np.pi/0.8], method = "brentq")
                is_success = True
                break
            except:
                print("\n\nError with brentq! trying new variable!  Old variable:", limits)

        if is_success == False:
            sys.exit()
            
        return solution.root
    
    def _dt_given_flex_joint_angle(self, angle):
        dt = self._flex_dt_given_theta(angle, self.hand_config["constants"]["joint_length"], self.hand_config["constants"]["tendon_lever"], self.hand_config["constants"]["length_wo_tendon"])
        return dt
    
    def _dt_given_pin_joint_angle(self, angle, finger_name, joint_name):
        finger_radius = self.hand_config[finger_name]["radius"][joint_name]
        dt = angle * finger_radius

        return dt
    
    def _calculate_pin_joint_angle_from_tendon(self, tendon_pos, finger_name, joint_name):
        finger_radius = self.hand_config[finger_name]["radius"][joint_name]

        joint_name_list = [self.hand_config[finger_name]["joint_name"][joint_name]]
        joint_angle_list = [(tendon_pos / finger_radius)]

        return joint_name_list, joint_angle_list
                   
    def _calculate_double_flexure_joint_angle_from_tendon(self, tendon_pos, finger_name):
        joint_name_list = self.hand_config[finger_name]["joint_name"]["flex"]
        joint_angle_list = []
    
        stiffnss_config = self.hand_config[finger_name]["stiffness_ratio"]

        for stiffness in stiffnss_config.values():
            ratio = stiffness / sum(stiffnss_config.values())
            tendon_position_for_joint = tendon_pos * ratio

            angle = self._theta_given_flex_dt(tendon_position_for_joint, self.hand_config["constants"]["joint_length"], 
                                               self.hand_config["constants"]["tendon_lever"], self.hand_config["constants"]["length_wo_tendon"])
            
            joint_angle_list.append(angle)

        return joint_name_list, joint_angle_list

    def _calculate_single_flexure_joint_angle_from_tendon(self, tendon_pos, finger_name, joint_name):     
        angle = self._theta_given_flex_dt(tendon_pos, self.hand_config["constants"]["joint_length"], 
                                            self.hand_config["constants"]["tendon_lever"], self.hand_config["constants"]["length_wo_tendon"])
        
        joint_name_list = [self.hand_config[finger_name]["joint_name"][joint_name]]    
        joint_angle_list = [angle]

        return joint_name_list, joint_angle_list
    
    def _calculate_spread_coupled_joint_angle_from_tendon(self, servo_pos):
        motion_ratio = self.hand_config["Spread"]["motion_ratio"]
        joint_name_list = self.hand_config["Spread"]["joint_name"]
        joint_angle_list = []

        for joint_name in joint_name_list:
            
            if "Index" in joint_name:
                joint = servo_pos * self.spread_angle_correction * motion_ratio["Index"]
            elif "Ring" in joint_name:
                joint = servo_pos * self.spread_angle_correction * motion_ratio["Ring"]
            elif "Pinky" in joint_name:
                joint = servo_pos * self.spread_angle_correction * motion_ratio["Pinky"]
            else:
                joint = 0.0
            
            joint_angle_list.append(joint)
    
        return joint_name_list, joint_angle_list
    
    # Public methods
    def servo_pos_to_robot_joint_pos(self, servo_pos:JointState):
        hand_joint_information = {"name":[], "angle":[]}
        vsa_data = None
        wrist_data = None

        for name, pos in zip(servo_pos.name, servo_pos.position):
            key = name.split("_")

            if "VSA" in name:
                if vsa_data is None:
                    vsa_data = {}

                vsa_data[name] = pos
                continue

            if "Wrist" in name:
                if wrist_data is None:
                    wrist_data = {}

                wrist_data[name] = pos
                continue

            _, _, _, motor_radius, _ = self.motor_regulator.get_motor_properties_from_motion_name(name)
            tendon_position = pos * (np.pi/180) * motor_radius

            if "Spread" in name:
                joint_name_list, joint_angle_list = self._calculate_spread_coupled_joint_angle_from_tendon(pos)
            
            elif "Thumb" in name:
                if "CMC" in name:
                    joint_name_list, joint_angle_list = self._calculate_pin_joint_angle_from_tendon(tendon_position, key[0], key[1])
                else:
                    joint_name_list, joint_angle_list = self._calculate_single_flexure_joint_angle_from_tendon(tendon_position, key[0], key[1])
            else:
                if "MCP" in name:
                    joint_name_list, joint_angle_list = self._calculate_pin_joint_angle_from_tendon(tendon_position, key[0], key[1])
                else:
                    joint_name_list, joint_angle_list = self._calculate_double_flexure_joint_angle_from_tendon(tendon_position, key[0])

            hand_joint_information["name"] += joint_name_list
            hand_joint_information["angle"] += joint_angle_list

        # Collect wrist info
        if wrist_data is not None:
            pitch, yaw = self.wrist_servo_pos_to_pitch_yaw(wrist_data["Wrist_Thumb_Side"], wrist_data["Wrist_Pinky_Side"])
            hand_joint_information["name"] += ["Wrist_Pitch", "Wrist_Yaw"]
            hand_joint_information["angle"] += [np.radians(pitch), np.radians(yaw)]

        # Convert everything to deg
        for i in range(len(hand_joint_information["name"])):
            hand_joint_information["angle"][i] = np.degrees(hand_joint_information["angle"][i])

        # Get vsa data
        if vsa_data is not None:
            vsa_ratio = self.servo_pos_to_vsa_pos(vsa_data["VSA1"], vsa_data["VSA2"])
            hand_joint_information["name"] += ["VSA"]
            hand_joint_information["angle"] += [vsa_ratio]
                        
        joint_state_msg = JointState()
        joint_state_msg.name = hand_joint_information["name"]
        joint_state_msg.position = hand_joint_information["angle"]

        return hand_joint_information, joint_state_msg
    
    def real_est_to_sim(self, real):
        sim = cp(real)
        for i, name in enumerate(real["name"]):
            params = self.hand_config["sim params"][name]
            sim["angle"][i]  = np.radians(real["angle"][i]) * params["dir"] + np.radians(params["offset"])

        joint_state_msg = JointState()
        joint_state_msg.name = sim["name"]
        joint_state_msg.position = sim["angle"]

        return joint_state_msg
    
    def _thumb_mcp_ip_compensation(self, mcp, ip):
        mcp = mcp * (mcp*(1/90)* 0.6 + 1)
        ip = ip * (ip*(1/90)* 0.6 + 1)
        return mcp, ip

    def hand_pos_to_servo_pos(self, hand_pos:dict):
        servo_pos = {}
        # Fingers
        for finger_name in ["Index", "Middle", "Ring", "Pinky"]:
            servo_pos[finger_name + "_MCP"] = self._dt_given_pin_joint_angle(np.radians(hand_pos[finger_name + "_MCP"]), finger_name, "MCP")

            pip_dt = self._dt_given_flex_joint_angle(np.radians(hand_pos[finger_name + "_PIP"]))
            dip_dt = self._dt_given_flex_joint_angle(np.radians(hand_pos[finger_name + "_DIP"]))

            servo_pos[finger_name + "_PIP_DIP"] = pip_dt + dip_dt

        # Thumb
        servo_pos["Thumb_CMC1"] = self._dt_given_pin_joint_angle(np.radians(hand_pos["Thumb_CMC1"]), "Thumb", "CMC1")
        servo_pos["Thumb_CMC2"] = self._dt_given_pin_joint_angle(np.radians(hand_pos["Thumb_CMC2"]), "Thumb", "CMC2")

        # TODO: Correlate this dual contribution with real robot
        compensated_mcp, compensated_ip = self._thumb_mcp_ip_compensation(hand_pos["Thumb_MCP"], hand_pos["Thumb_IP"])

        mcp_dt = self._dt_given_flex_joint_angle(np.radians(compensated_mcp))
        ip_dt = self._dt_given_flex_joint_angle(np.radians(compensated_ip))

        servo_pos["Thumb_MCP"] = mcp_dt
        servo_pos["Thumb_IP"] = ip_dt

        for key in servo_pos.keys():
            _, _, _, motor_radius, _ = self.motor_regulator.get_motor_properties_from_motion_name(key)
            servo_pos[key] = float(servo_pos[key] * (180/np.pi) / motor_radius)

        # Calculate spread position
        # index_spread = min(1, max(0, (hand_pos["Index_MCP_Spread"] + 12) / 18) )
        # ring_spread = min(1, max(0, -1 * (hand_pos["Ring_MCP_Spread"] - 4) / 14) )
        # pinky_spread = min(1, max(0, -1 * (hand_pos["Pinky_MCP_Spread"] - 8) / 28) )

        index_spread = min(1, max(0, hand_pos["Index_MCP_Spread"]))
        ring_spread = min(1, max(0, (hand_pos["Ring_MCP_Spread"])))
        pinky_spread = min(1, max(0, (hand_pos["Pinky_MCP_Spread"])))

        total_spread = index_spread * 0.6 + ring_spread * 0.2 + pinky_spread * 0.2

        _, _, limits, _, _ = self.motor_regulator.get_motor_properties_from_motion_name("Finger_MCP_Spread")

        servo_pos["Finger_MCP_Spread"] = float(limits[1]*total_spread)

        return servo_pos
    
    def wrist_servo_pos_to_pitch_yaw(self, thumb_side_angle, pinky_side_angle):
        pitch_amount_raw = np.mean([thumb_side_angle, pinky_side_angle])
        yaw_amount_raw = thumb_side_angle - pinky_side_angle

        pitch = pitch_amount_raw * self.hand_config["Wrist"]["spur_ratio"]
        yaw = yaw_amount_raw * self.hand_config["Wrist"]["spur_ratio"] * self.hand_config["Wrist"]["bevel_ratio"]
        
        return pitch, yaw
    
    def pitch_yaw_to_wrist_servo_pos(self, pitch_angle, yaw_angle):
        a = pitch_angle / self.hand_config["Wrist"]["spur_ratio"]
        b = yaw_angle / (2 * self.hand_config["Wrist"]["spur_ratio"] * self.hand_config["Wrist"]["bevel_ratio"])
        
        thumb_side_angle = a + b
        pinky_side_angle = a - b

        return thumb_side_angle, pinky_side_angle

    def limit_pitch_yaw(self, pitch, yaw):
        pitch_lim = self.hand_config["Wrist"]["limits"]["Pitch"]
        yaw_lim = self.hand_config["Wrist"]["limits"]["Yaw"]

        is_valid = True

        if pitch < pitch_lim[0]:
            pitch = pitch_lim[0]; is_valid = False

        if pitch > pitch_lim[1]:
            pitch = pitch_lim[1]; is_valid = False

        if yaw < yaw_lim[0]:
            yaw = yaw_lim[0]; is_valid = False

        if yaw > yaw_lim[1]:
            yaw = yaw_lim[1]; is_valid = False

        return pitch, yaw, is_valid


    def vsa_demand_to_servo_demand(self, demand):
        # Limit the demand
        demand = max(0, min(1, demand))

        # Get maximum value
        _, _, limits1, _, _ = self.motor_regulator.get_motor_properties_from_motion_name("VSA1")
        _, _, limits2, _, _ = self.motor_regulator.get_motor_properties_from_motion_name("VSA2")

        pos1 = max(limits1) * demand; pos2 = max(limits2) * demand

        return pos1, pos2
    
    def servo_pos_to_vsa_pos(self, vsa1, vsa2):
        _, _, limits1, _, _ = self.motor_regulator.get_motor_properties_from_motion_name("VSA1")
        _, _, limits2, _, _ = self.motor_regulator.get_motor_properties_from_motion_name("VSA2")

        ratio1 = vsa1/max(limits1); ratio2 = vsa2/max(limits2)

        return float((ratio1 + ratio2)/2)