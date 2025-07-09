from scipy.spatial.transform import Rotation as R
import numpy as np
from copy import deepcopy as cp
from adapt_driver.hand_regulator import HandRegulator
from adapt_driver.motor_regulator import MotorRegulator
from scipy.spatial.transform import Slerp



def get_relative_quaternion(q1, q2):
    r1 = R.from_quat(q1)
    r2 = R.from_quat(q2)
    return r1.inv() * r2


class AVPTeleopWrist():
    def __init__(self):
        super().__init__()
        self.static_avp_t = None
        self.current_franka_pose = None
        self.current_pitch = None
        self.current_yaw = None

        self.hand_regulator = HandRegulator()
        self.motor_regulator = MotorRegulator()


    def arm_pos_receiver(self, msg):
        self.current_franka_pose = msg

    def gamepad_callback(self, gamepad_raw_msg):
        # print("self.gp.button_data[L1] ", self.gp.button_data["L1"])
        self.gp.convert_joy_msg_to_dictionary(gamepad_raw_msg)

        if self.is_pressed == False and self.gp.button_data["L1"] == 1: # need to change this to a toggle button
            self.reset_pose()
            self.is_pressed = True
        if self.gp.button_data["L1"] == 0: # need to change this to a toggle button
            self.is_pressed = False
    
    def adapt_right_wrist_receiver(self, joint_pitch, joint_yaw):
        #  "Wrist_Pitch" joint
        self.current_pitch = joint_pitch
        # "Wrist_Yaw" joint 
        self.current_yaw = joint_yaw
    
    def reset_pose(self):
        self.static_avp_t = cp(self.avp_wrist_t)
        self.static_franka_pose = cp(self.current_franka_pose)
        self.simulated_robot_pose = cp(self.current_franka_pose)
    
    def avp_demand_receiver(self, wrist_pose):
        wrist_pose_data = wrist_pose
        raw_t = np.array([wrist_pose_data[0], wrist_pose_data[1], wrist_pose_data[2]])
        raw_q = np.array([wrist_pose_data[3], wrist_pose_data[4], wrist_pose_data[5], wrist_pose_data[6]])

        # base_rot = R.from_euler("z", -90, degrees=True)
        base_rot = R.from_euler("z", 0, degrees=True) # rotation between the AVP and the robot arm base frame
        base_rot_m = base_rot.as_matrix()

        self.avp_wrist_t = np.matmul(base_rot_m, raw_t)
        avp_wrist_m = np.matmul(base_rot_m, R.from_quat(raw_q).as_matrix())
        self.avp_wrist_q = R.from_matrix(avp_wrist_m).as_quat()

        if self.static_avp_t is None:
            self.reset_pose()

        self.translation_demand = []
        for i in range(3):
            self.translation_demand.append(self.avp_wrist_t[i] - self.static_avp_t[i] + self.static_franka_pose[i])
        print("translation_demand ", self.translation_demand)

        print("")
    
    
    def hand_to_arm(self, hand_coord):
        # convert hand base (amount point) coordinates to arm coordinates
        output = cp(hand_coord)
        ee_m = R.from_quat(output[3:]).as_matrix()

        adjust_rot_m1 = R.from_euler("y", -90, degrees=True).as_matrix()
        adjust_rot_m2 = R.from_euler("z", -135, degrees=True).as_matrix()
        hand_rotated_m = np.matmul(np.matmul(ee_m, adjust_rot_m1), adjust_rot_m2)

        trans_vec = np.matmul(hand_rotated_m, [0, 0, -0.1])
        hand_q = R.from_matrix(hand_rotated_m).as_quat()

        for i in range(3):
            output[i] += trans_vec[i]

        for i in range(4):
            output[3+i] = hand_q[i]

        return output

    def arm_to_hand_base(self, arm_coord):
        # convert arm coordinates to hand base (amount point) coordinates
        output = cp(arm_coord)

        ee_m = R.from_quat(output[3:]).as_matrix()
        trans_vec = np.matmul(ee_m, [0, 0, 0.1])

        adjust_rot_m1 = R.from_euler("z", 135, degrees=True).as_matrix()
        adjust_rot_m2 = R.from_euler("y", 90, degrees=True).as_matrix()
        hand_m = np.matmul(np.matmul(ee_m, adjust_rot_m1), adjust_rot_m2)
        hand_q = R.from_matrix(hand_m).as_quat()

        for i in range(3):
            output[i] += trans_vec[i]

        for i in range(4):
            output[3+i] = hand_q[i]

        return output

    def hand_base_to_wirst(self, hand_base):
        # convert hand base coordinates (amount point) to wrist coordinates
        ee_m = R.from_quat(hand_base[3:]).as_matrix()

        pitch_m = R.from_euler("z", self.current_pitch, degrees=True).as_matrix() 
        yaw_m = R.from_euler("y", self.current_yaw, degrees=True).as_matrix() 
        wrist_m = np.matmul(np.matmul(ee_m, pitch_m), yaw_m)
        wrist_q = R.from_matrix(wrist_m).as_quat()

        return wrist_q

    def command_wrist(self, extra_pitch ,extra_yaw):

        # Determine the pich and yaw that can be applied based on limits
        demand_pitch, demand_yaw, _ = self.hand_regulator.limit_pitch_yaw(self.current_pitch + extra_pitch, 
                                                                                self.current_yaw + extra_yaw)
        
        thumb_side, pinky_side = self.hand_regulator.pitch_yaw_to_wrist_servo_pos(demand_pitch, demand_yaw)
        motor_demand_limited = self.motor_regulator.correct_motor_limits({"Wrist_Thumb_Side":thumb_side,
                                                                          "Wrist_Pinky_Side":pinky_side})
    
        pitch_limited, yaw_limited = self.hand_regulator.wrist_servo_pos_to_pitch_yaw(motor_demand_limited["Wrist_Thumb_Side"], motor_demand_limited["Wrist_Pinky_Side"])

        return pitch_limited, yaw_limited


    def get_wrist_arm_coord(self, ee_m, pitch, yaw):
        pitch_m = R.from_euler("z", pitch, degrees=True).as_matrix()
        raw_m = R.from_euler("y", yaw, degrees=True).as_matrix()
        wrist_m = np.matmul(np.matmul(ee_m, pitch_m), raw_m)
        wrist_q = R.from_matrix(wrist_m).as_quat()

        residual_m = get_relative_quaternion(wrist_q, self.avp_wrist_q).as_matrix()

        residual_wrt_ee_m = np.matmul(ee_m, residual_m)
        residual_wrt_ee_q = R.from_matrix(residual_wrt_ee_m).as_quat()

        return wrist_q, residual_wrt_ee_q


    def get_arm_wrist_command(self):
        starting_franka_pose = cp(self.current_franka_pose)

        hand_base = self.arm_to_hand_base(self.current_franka_pose)
        current_hand_trans = hand_base[:3]
        current_hand_quat = self.hand_base_to_wirst(hand_base)

        hand_transform = []
        for i in range(3):
            hand_transform.append(current_hand_trans[i])
        for i in range(4):
            hand_transform.append(current_hand_quat[i])

        diff = get_relative_quaternion(current_hand_quat, self.avp_wrist_q).as_euler("zyx", degrees = True) # diff of orientation between the current hand and the AVP wrist pose
        applied_pitch, applied_yaw = self.command_wrist(diff[0], diff[1])
        wrist_q, residual_q = self.get_wrist_arm_coord(R.from_quat(hand_base[3:]).as_matrix(), applied_pitch, applied_yaw)


        panda_demand = self.hand_to_arm([current_hand_trans[0], current_hand_trans[1], current_hand_trans[2], # residual orientation of hand wrist -> movement of the arm (used for the residual hand wrist orientation)
                                         residual_q[0], residual_q[1], residual_q[2], residual_q[3]])
        
        trans_demand = np.asarray(self.translation_demand) + np.asarray(panda_demand[:3]) - np.asarray(starting_franka_pose[:3])

        # movement target for the arm
        trans_delta = trans_demand - np.asarray(starting_franka_pose[:3])
        rot_delta = get_relative_quaternion(panda_demand[3:], starting_franka_pose[3:])

        dt = 0.05
        trans_speed = np.linalg.norm(trans_delta)/dt
        rot_speed = np.degrees(rot_delta.magnitude())/dt

        base = 0.3; reach = -2; max_damp = 0.5
        trans_damp = min(base + (1-base) * (1- np.exp(reach * trans_speed)), max_damp)

        base = 0.7; reach = -2; max_damp = 0.8
        rot_damp = min(base + (1-base) * (1- np.exp(reach * rot_speed)), max_damp)

        slerp = Slerp([0, 1], R.from_quat([panda_demand[3:], starting_franka_pose[3:]]))
        target_R = slerp([rot_damp])
        target_hand_orientation = target_R.as_quat()[0]

        franka_demand = []
        for i in range(7):
            if i < 3:
                franka_demand.append(starting_franka_pose[i] * trans_damp + trans_demand[i] * (1-trans_damp))
            else:
                franka_demand.append(target_hand_orientation[i-3])
        if self.gp.button_data["L1"] == 1:
            return franka_demand, applied_pitch, applied_yaw,
        else:
            return starting_franka_pose, self.current_pitch, self.current_yaw, hand_transform
    