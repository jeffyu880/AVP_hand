import os, sys, json, time
from copy import deepcopy as cp

from .utility import get_robot_id

sys.path.append(os.path.join(os.path.dirname(__file__), "../dynamixel"))

class MotorRegulator:
    def __init__(self) -> None:
        # Get configuration ID
        id = get_robot_id("../robot_config")

        dir_path = os.path.dirname(os.path.realpath(__file__))        

        json_file = dir_path + "/../robot_config/" + id + "/motor.config.json"
        self.config = json.load(open(json_file))
        
        self.all_motion_names = []
        for value in self.config.values():
            self.all_motion_names.append(value["motion_name"])

    def __get_motor_config(self, dynamixel_name):
            zero_angle = self.config[dynamixel_name]["zero_angle"]
            if self.config[dynamixel_name]["direction"] == "cw":
                direction = -1
            else:
                direction = 1
            limits = self.config[dynamixel_name]["angle_limit"]
            radius = self.config[dynamixel_name]["radius"]
            decay = self.config[dynamixel_name]["decay"]

            return zero_angle, direction, limits, radius, decay
    
    def get_motor_properties_from_motion_name(self, motion_name):
        for key, value in self.config.items():
            if value["motion_name"] == motion_name:
                zero_angle, direction, limits, radius, decay = self.__get_motor_config(key)
                break

        return zero_angle, direction, limits, radius, decay

    def get_empty_motor_pos_dict(self):
        motor_pos_dict = {}
        for value in self.config.values():
            motor_pos_dict[value["motion_name"]] = 0

        return motor_pos_dict
    
    def get_default_motor_setting(self):
        default_current_mA_setting = {}
        default_profile_velocity_deg_setting = {}

        for value in self.config.values():
            default_current_mA_setting[value["motion_name"]] = value["default_current_mA"]
            default_profile_velocity_deg_setting[value["motion_name"]] = value["default_speed_deg"]
        
        return default_current_mA_setting, default_profile_velocity_deg_setting
        
    def correct_motor_limits(self, demands:dict):
        limit_applied_demand = cp(demands)
        for motion_name, demand in demands.items():
            _, _, limits, _, _ = self.get_motor_properties_from_motion_name(motion_name)

            if demand < limits[0]:
                limit_applied_demand[motion_name] = float(cp(limits[0]))
            
            if demand > limits[1]:
                limit_applied_demand[motion_name] = float(cp(limits[1]))

        return limit_applied_demand
    
    def treated_to_raw_motor_pos(self, treated: dict):
        raw = {}
        for motion_name in treated.keys():
            treated_angle = treated[motion_name]
            zero_angle, direction, _, _, _ = self.get_motor_properties_from_motion_name(motion_name)

            raw[motion_name] = (treated_angle * direction) + zero_angle

        return raw

    def raw_motor_pos_vel_to_treated(self, raw_pos: dict, raw_vel: dict):
        treated_pos = {}
        treated_vel = {}
        for motion_name in raw_pos.keys():
            zero_angle, direction, _, _, _ = self.get_motor_properties_from_motion_name(motion_name)

            treated_pos[motion_name] = (raw_pos[motion_name] - zero_angle) * direction
            treated_vel[motion_name] = raw_vel[motion_name] * direction

        return treated_pos, treated_vel
    

    def apply_decay(self, demand):
        demand_with_decay = cp(demand)
        limited_demand = self.correct_motor_limits(demand)
        for motion_name in self.current_treated_servo_data["position"].keys():
            _, _, _, _, decay = self.get_motor_properties_from_motion_name(motion_name)
            demand_with_decay[motion_name] = decay * self.current_treated_servo_data["position"][motion_name] + (1-decay) * limited_demand[motion_name]

        return demand_with_decay
    
    def write_position(self, demand, is_sim = False):
        servo_demand = self.correct_motor_limits(demand)

        if is_sim:
            for motion in self.active_motions:
                self.sim_servo_pos[motion] = cp(servo_demand[motion])

        else:
            motor_demand = self.treated_to_raw_motor_pos(servo_demand)

            positions = []
            for motion in self.active_motions:
                positions.append(motor_demand[motion])
            
            self.ctrl.set_goal_position_deg(positions)
