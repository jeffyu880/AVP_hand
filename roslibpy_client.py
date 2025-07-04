import roslibpy
import sys
import numpy as np
import time
from copy import deepcopy as cp
from scipy.interpolate import interp1d

class AdaptClient:
    def __init__(self, ip, port = 9090, has_vsa = False, has_tactile_sense = False, has_spring_sense = False) -> None:
        # options
        self.has_vsa = has_vsa
        self.has_tactile_sense = has_tactile_sense
        self.has_spring_sense = has_spring_sense    
    
        self.client = roslibpy.Ros(host = ip, port = port)
        self.client.run()

        if self.client.is_connected:
            print("Successfully connected to ADAPT Hand")
        else:
            print("Connection failed, try again :(")
            sys.exit()

        # Publishers
        self.servo_demand_publisher = roslibpy.Topic(self.client, '/adapt/hand/servo_demand', 'sensor_msgs/JointState')
        self.joint_demand_publisher = roslibpy.Topic(self.client, '/adapt/hand/joint_demand', 'sensor_msgs/JointState')

        if self.has_vsa:
            self.vsa_demand_publisher = roslibpy.Topic(self.client, '/adapt/vsa/demand', 'std_msgs/Float64')

        # Subscribers
        self.servo_data_subscriber = roslibpy.Topic(self.client, '/adapt/current_servo_data', 'sensor_msgs/JointState')
        self.servo_data_subscriber.subscribe(self.servo_data_callback)

        self.est_joints_subscriber = roslibpy.Topic(self.client, '/adapt/est_joint_states', 'sensor_msgs/JointState')
        self.est_joints_subscriber.subscribe(self.est_joints_callback)

        if self.has_spring_sense:
            self.spring_data_subscriber = roslibpy.Topic(self.client, '/adapt/sensing/spring', 'std_msgs/Float64MultiArray')
            self.spring_data_subscriber.subscribe(self.spring_data_callback)

        if self.has_tactile_sense:
            self.tactile_data_subscriber = roslibpy.Topic(self.client, '/adapt/sensing/tactile', 'std_msgs/Float64MultiArray')
            self.tactile_data_subscriber.subscribe(self.tactile_data_callback)

        # Names
        # self.hand_servo_names = ["Index_MCP", "Index_PIP_DIP", "Middle_MCP", "Middle_PIP_DIP",
        #                            "Ring_MCP", "Ring_PIP_DIP", "Pinky_MCP", "Pinky_PIP_DIP",
        #                            "Thumb_CMC1", "Thumb_CMC2", "Thumb_MCP", "Thumb_IP", "Finger_MCP_Spread"]
        
        # self.hand_joint_names = ["Index_MCP", "Index_MCP_Spread", "Index_PIP", "Index_DIP", 
        #                          "Middle_MCP", "Middle_MCP_Spread", "Middle_PIP", "Middle_DIP",
        #                          "Ring_MCP", "Ring_MCP_Spread", "Ring_PIP", "Ring_DIP",
        #                          "Pinky_MCP", "Pinky_MCP_Spread", "Pinky_PIP", "Pinky_DIP",
        #                          "Thumb_CMC1", "Thumb_CMC2", "Thumb_MCP", "Thumb_IP"]
        
        # Initial values
        while hasattr(self, "current_servo_data") == False:
            pass        

        while self.has_spring_sense and (hasattr(self, "raw_spring_data") == False):
            pass

        while self.has_tactile_sense and (hasattr(self, "raw_tactile_data") == False):
            pass

    def set_csv_location(self, loc):
        self.csv_directory = loc

    def show_topics(self):
        topics = self.client.get_topics()
        for topic in topics:
            print(topic)
    
    def write_vsa_angle(self, angle):
        angle = max(10, min(90, angle))
        m = interp1d([10,90],[0,1])

        demand = max(0, min(1, m(angle)))

        self.vsa_demand_publisher.publish(roslibpy.Message({'data': float(demand)}))

    def est_joints_callback(self, msg):
        self.est_joint_data = msg
        self.est_joint_positions = dict(zip(list(msg["name"]), list(msg["position"])))

    def servo_data_callback(self, msg):
        self.current_servo_data = msg
        self.current_servo_position = dict(zip(list(msg["name"]), list(msg["position"])))

    def spring_data_callback(self, msg):
        self.raw_spring_data = msg["data"]

    def tactile_data_callback(self, msg):
        self.raw_tactile_data = msg["data"]

    def write_servo_demand(self, demand:dict):
        motion_names = list(demand.keys())
        positions = list(demand.values())

        message = roslibpy.Message({
            "header": {"stamp": { "sec": 0,"nsec": 0 },"frame_id": ""},
            "name":motion_names,
            "position":positions
            })

        self.servo_demand_publisher.publish(message)

    def write_joint_demand(self, demand:dict):
        all_joint_demand = cp(self.est_joint_positions)

        for key, value in demand.items():
            try:
                all_joint_demand[key] = value
            except:
                pass
                
        joint_names = list(all_joint_demand.keys())
        positions = list(all_joint_demand.values())

        message = roslibpy.Message({
            "header": {"stamp": { "sec": 0,"nsec": 0 },"frame_id": ""},
            "name":joint_names,
            "position":positions
            })

        self.joint_demand_publisher.publish(message)

    def load_pose_csv(self, csv_name):
        data = np.loadtxt(self.csv_directory + csv_name, delimiter=",", dtype=str)
        
        target_names = np.char.strip(data[0,:])
        target_values = data[1:, :].astype(float)

        return target_names, target_values

    def interpolate_dictionaries(self, names, start, end, percentage):
        demand = {}
        for i, name in enumerate(names):
            diff = end[i] - start[i]
            demand[name] = diff * percentage + start[i]

        return demand
    
    def compute_vel_params(self, start_pose, end_pose, target_vel = 50, minimum_time = 0.1, dt = 0.01):
        
        max_servo_disp = -1
        for i in range(len(start_pose)):
            servo_disp = abs(end_pose[i] - start_pose[i])
            if servo_disp > max_servo_disp:
                max_servo_disp = servo_disp

        t = max(max_servo_disp/target_vel, minimum_time)
        num_of_divisions = round(t/dt)

        return dt, num_of_divisions
    
    def playback_trajectory(self, csv_name):
        target_names, waypoints = self.load_pose_csv(self.csv_directory + csv_name)

        # Append initial traj
        current_value = []
        for target_name in target_names:
            current_value.append(self.current_servo_position[target_name])

        waypoints = np.vstack((current_value, waypoints))     

        for i in range(len(waypoints) - 1):
            start_wp = waypoints[i]
            end_wp = waypoints[i+1]

            # Calculate velocity params
            dt, num_divisions = self.compute_vel_params(start_wp, end_wp)

            for div in range(num_divisions):
                percentage = div/num_divisions
                demand = self.interpolate_dictionaries(target_names, start_wp, end_wp, percentage)

                self.write_servo_demand(demand)

                time.sleep(dt)

    def interpolate_multiple_waypoints(self, csv_name, percentage):
        target_names, waypoints = self.load_pose_csv(self.csv_directory + csv_name)

        divisions = len(waypoints) - 1
        div_size = 1/divisions
        percentage = max(0, min(0.99999999, percentage))

        start_idx = int(np.floor(percentage / (1/divisions)))

        playback_percentage = (percentage - start_idx * div_size) / div_size
        
        demand = self.interpolate_dictionaries(target_names, waypoints[start_idx], waypoints[start_idx + 1], playback_percentage)

        return demand