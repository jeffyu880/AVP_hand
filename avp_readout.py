from scipy.spatial.transform import Rotation as R
from avp_stream import VisionProStreamer
from helper_functions import get_hand_joints



class AVP:
    def __init__(self, ip):
        # avp related
        self.avp = VisionProStreamer(ip = ip, record = True)
        self.latest = {}


    def populate_latest(self, frame_name, data):
        rot_matrix = data[:3,:3]
        trans = data[:3, 3]

        quat = R.as_quat(R.from_matrix(rot_matrix))

        self.latest[frame_name] = [trans, quat, rot_matrix]

        # self.latest: dict_keys(['wrist_r', 'Base_r', 'Thumb_CMC_r', 'Thumb_MCP_r', 'Thumb_IP_r', 'Thumb_TIP_r', 'Index_Base_r', 'Index_MCP_r', 'Index_PIP_r', 'Index_DIP_r', 'Index_TIP_r', 'Middle_Base_r', 'Middle_MCP_r', 'Middle_PIP_r', 'Middle_DIP_r', 'Middle_TIP_r', 'Ring_Base_r', 'Ring_MCP_r', 'Ring_PIP_r', 'Ring_DIP_r', 'Ring_TIP_r', 'Pinky_Base_r', 'Pinky_MCP_r', 'Pinky_PIP_r', 'Pinky_DIP_r', 'Pinky_TIP_r'])

    

    def get_frame(self):
        frame = self.avp.latest # dict_keys(['left_wrist', 'right_wrist', 'left_fingers', 'right_fingers', 'head', 'left_pinch_distance', 'right_pinch_distance', 'right_wrist_roll', 'left_wrist_roll'])

        self.populate_latest("wrist_r", frame["right_wrist"][0])
        self.populate_latest("wrist_l", frame["left_wrist"][0])

        fingers = ["Base", "Thumb_CMC", "Thumb_MCP", "Thumb_IP", "Thumb_TIP",
                   "Index_Base", "Index_MCP", "Index_PIP", "Index_DIP", "Index_TIP",
                   "Middle_Base", "Middle_MCP", "Middle_PIP", "Middle_DIP", "Middle_TIP",
                   "Ring_Base", "Ring_MCP", "Ring_PIP", "Ring_DIP", "Ring_TIP",
                   "Pinky_Base", "Pinky_MCP", "Pinky_PIP", "Pinky_DIP", "Pinky_TIP"]

        for i, name in enumerate(fingers):
            if name != "none":
                self.populate_latest(name + "_r", frame["right_fingers"][i])

        for i, name in enumerate(fingers):
            if name != "none":
                self.populate_latest(name + "_l", frame["left_fingers"][i])

        self.right_est_hand_joints = get_hand_joints(self.latest, side="r")
        self.left_est_hand_joints = get_hand_joints(self.latest, side="l")



        #################################################################

        self.right_wrist_pose = list(self.latest["wrist_r"][0]) + list(self.latest["wrist_r"][1])
        self.left_wrist_pose = list(self.latest["wrist_l"][0]) + list(self.latest["wrist_l"][1])

        def fmt_pose(pose):
            roll, yaw, pitch = R.from_quat(pose[3:]).as_euler("zyx", degrees=True)
            return f"roll: {roll:.1f}, pitch: {pitch:.1f}, yaw: {yaw:.1f}"

        print("LEFT WRIST: ", fmt_pose(self.left_wrist_pose))       # prints, x, y, z, qx, qy, qz, qw
        print("RIGHT WRIST:", fmt_pose(self.right_wrist_pose))

        # print("self.avp.latest:", self.avp.latest)

        # print("self.latest:", self.latest)
        # print("self.est_hand_pos:", self.est_hand_pos)
        # print("wrist_pose:", wrist_pose)

        # print("self.avp.latest keys:", self.avp.latest.keys())
        # print("self.latest:", self.latest.keys())
        # print("self.est_hand_pos:", self.est_hand_pos.keys())




        