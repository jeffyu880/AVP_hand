import roslibpy
from roslibpy import Message
from helper_functions.gamepad_functions import GamepadFunctions
import time


class Bridge_control:

    def __init__(self) -> None:

        # self.ros_client = roslibpy.Ros(host='localhost', port=9090)
        self.ros_client = roslibpy.Ros(host='10.42.0.144', port=9090)  # To Change
        self.ros_client.run()

        self.gp = GamepadFunctions()
        # # Publishers -----------------------------------------

        self.franka_pose_publisher = roslibpy.Topic(self.ros_client, '/wrist/demand', 'std_msgs/Float64MultiArray')
        self.franka_pose_publisher.advertise()

        
        self.hand_servo_mt_publisher = roslibpy.Topic(self.ros_client, '/adapt_1/hand/servo_demand', 'sensor_msgs/JointState') # '/hand/servo_demand'
        self.hand_servo_mt_publisher.advertise()
        self.target_hand_servo_names = ['Thumb_CMC2', 'Thumb_CMC1', 'Index_MCP', 'Middle_MCP', 'Ring_MCP', 'Pinky_MCP', 'Pinky_PIP_DIP', 'Ring_PIP_DIP', 'Middle_PIP_DIP', 'Index_PIP_DIP', 'Thumb_MCP', 'Thumb_IP', 'Finger_MCP_Spread']


    # Publisher Functions -----------------------------------------
    def publish_hand_servo_mt(self, mt_demands):
        # Create a message  

        message = roslibpy.Message({
            "header": {"stamp": { "sec": 0,"nsec": 0 },"frame_id": ""},
            "name":list(self.target_hand_servo_names) ,
            "position":list(mt_demands)
            })

        self.hand_servo_mt_publisher.publish(message)
        # print(f'Published hand mt demands: {mt_demands}')
        # print(f'Published hand mt demands: {len(mt_demands)}')
    

    def publish_franka_pose(self, pose):
        # Create a message
        message = roslibpy.Message({'data': pose})
        self.franka_pose_publisher.publish(message)
        # print(f'Published Franka pose: {pose}')


if __name__ == "__main__":

    bridge = Bridge_control()

    # bridge.publish_franka_pose(pose_demand)
    i = 0
    while i < 100:
        grasp_mtDemands = [0.0] * 13 # dummy values
        grasp_mtDemands[0] = 45
        grasp_mtDemands[0] = 45 + i

        ## NOTE: check robot_config / 1 / motor_config /"angle_limit" for the limits of the servos

        bridge.publish_hand_servo_mt(grasp_mtDemands)
        i += 1
        time.sleep(0.5)
        print(f'Published hand mt demands: {grasp_mtDemands}')
        


