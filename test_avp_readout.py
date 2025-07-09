from avp_readout import AVP
from avp_teleop_wrist import AVPTeleopWrist
import time
import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# from roslibpy_client import AdaptClient
from adapt_driver.avp_adapt import HandRegulator
import numpy as np


def main():
    # TODO: Change this ip address to whatever you see when you start the tracking streamer on the headset
    
    adapt_hand = HandRegulator()
    # vision_pro = AVP(ip = "172.20.10.4")
    vision_pro = AVP(ip = "192.168.0.100")

    # change the ip address to 'adapt-rpi.local' when using a raspberry pi on USB gadget mode
    # if running on localhost use '127.0.0.1'
    # client = AdaptClient(ip = '172.20.10.12')
    avp_teleopWrist = AVPTeleopWrist()


    while 1:
        vision_pro.get_frame()

        # input("Press enter for motion 1")
        # client.write_joint_demand(vision_pro.est_hand_pos)

        # input("Press enter for motion 2")
        # client.write_joint_demand({"Index_MCP":0.0, "Middle_PIP":0.0, "Thumb_IP":0.0})

        #20-dof human hand joint angles
        # print(vision_pro.est_hand_pos)

        # Example output:
        """{
        'Index_MCP_Spread': -12.525850185451247,
          'Index_MCP': 26.002794324883197, 
          'Index_PIP': 43.679276430766954, 
          'Index_DIP': 17.633151148217056, 
          'Middle_MCP_Spread': -7.560979025448258, 
          'Middle_MCP': -3.6350047549231093, 
          'Middle_PIP': 65.8488436886601, 
          'Middle_DIP': 28.486235329265927, 
          'Ring_MCP_Spread': 0.625756950976116, 
          'Ring_MCP': -7.929526686404046, 
          'Ring_PIP': 63.556234527536226, 
          'Ring_DIP': 23.284284261898833, 
          'Pinky_MCP_Spread': -1.4584933310761055, 
          'Pinky_MCP': 2.0804478494550938, 
          'Pinky_PIP': 46.42438637873392, 
          'Pinky_DIP': 24.800166891669505, 
          'Thumb_CMC1': 81.48721126027745, 
          'Thumb_CMC2': 41.168103622503516, 
          'Thumb_MCP': 16.53097632325627, 
          'Thumb_IP': 40.55562921254244}
        """

        # 13-dof servo angles
        # print(adapt_hand.hand_pos_to_servo_pos(vision_pro.est_hand_pos))
        

        print("   ")

 
        # Example output:
        """
        {'Index_MCP': 74.78316995240695, 
        'Index_PIP_DIP': 31.356977482360957, 
        'Middle_MCP': 46.69547928302068, 
        'Middle_PIP_DIP': 45.385119569750636, 
        'Ring_MCP': 24.65419788097364, 
        'Ring_PIP_DIP': 53.49246914789531, 
        'Pinky_MCP': 34.514250156475846, 
        'Pinky_PIP_DIP': 40.245847860990914, 
        'Thumb_CMC1': 158.59350336978952, 
        'Thumb_CMC2': 97.83494917105155, 
        'Thumb_MCP': 24.677013720202336, 
        'Thumb_IP': 22.890527744748212, 
        'Finger_MCP_Spread': 4.11766708026889}
        """

        # time.sleep(0.1) # <--- this is just here to throttle how much gets printed on the terminal
        time.sleep(0.01)

        print("wrist_pose ", vision_pro.wrist_pose)

        
        arm_pose = np.array([1,1,1, 1.0, 0, 0, 0])  # Dummy arm pose data
        current_pitch = 0.0  # Dummy current pitch data
        current_yaw = 0.0  # Dummy current
        wrist_pose_demand = vision_pro.wrist_pose

        avp_teleopWrist.arm_pos_receiver(arm_pose)
        avp_teleopWrist.adapt_right_wrist_receiver(current_pitch, current_yaw)
        avp_teleopWrist.avp_demand_receiver(wrist_pose_demand)

        arm_demand, hand_wrist_pitch, hand_wrist_yaw, hand_transform = avp_teleopWrist.get_arm_wrist_command()
        
        print("arm_demand ",arm_demand)  # robot arm target position and orientation
        print("hand_wrist_pitch  hand_wrist_yaw ", hand_wrist_pitch, hand_wrist_yaw) # adapt hand joint demand for wrist motors


if __name__ == "__main__":
    main()