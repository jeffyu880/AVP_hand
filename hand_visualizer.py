import cv2
import numpy as np
from avp_readout import AVP

W, H  = 1280, 720
SCALE = 800    # pixels per metre

# Isometric view: looking from front, tilted down 30°
_el = np.radians(30)
VIEW = np.array([
    [1,           0,            0],
    [0, np.cos(_el), -np.sin(_el)],
    [0, np.sin(_el),  np.cos(_el)],
])

RIGHT_CHAINS = [
    ["wrist_r", "Base_r", "Thumb_CMC_r", "Thumb_MCP_r", "Thumb_IP_r", "Thumb_TIP_r"],
    ["Base_r", "Index_Base_r",  "Index_MCP_r",  "Index_PIP_r",  "Index_DIP_r",  "Index_TIP_r"],
    ["Base_r", "Middle_Base_r", "Middle_MCP_r", "Middle_PIP_r", "Middle_DIP_r", "Middle_TIP_r"],
    ["Base_r", "Ring_Base_r",   "Ring_MCP_r",   "Ring_PIP_r",   "Ring_DIP_r",   "Ring_TIP_r"],
    ["Base_r", "Pinky_Base_r",  "Pinky_MCP_r",  "Pinky_PIP_r",  "Pinky_DIP_r",  "Pinky_TIP_r"],
]

LEFT_CHAINS = [
    ["wrist_l", "Base_l", "Thumb_CMC_l", "Thumb_MCP_l", "Thumb_IP_l", "Thumb_TIP_l"],
    ["Base_l", "Index_Base_l",  "Index_MCP_l",  "Index_PIP_l",  "Index_DIP_l",  "Index_TIP_l"],
    ["Base_l", "Middle_Base_l", "Middle_MCP_l", "Middle_PIP_l", "Middle_DIP_l", "Middle_TIP_l"],
    ["Base_l", "Ring_Base_l",   "Ring_MCP_l",   "Ring_PIP_l",   "Ring_DIP_l",   "Ring_TIP_l"],
    ["Base_l", "Pinky_Base_l",  "Pinky_MCP_l",  "Pinky_PIP_l",  "Pinky_DIP_l",  "Pinky_TIP_l"],
]


def project(rel, pc, flip_x=False, flip_y=False):
    v = VIEW @ np.asarray(rel, dtype=float)
    sx = -1 if flip_x else 1
    sy =  1 if flip_y else -1
    return (int(pc[0] + sx * v[0] * SCALE),
            int(pc[1] + sy * v[1] * SCALE))


def draw_hand(img, latest, chains, wrist_key, color, pc, flip_x=False, flip_y=False):
    if wrist_key not in latest:
        return
    wrist_pos = latest[wrist_key][0]

    for chain in chains:
        valid = [j for j in chain if j in latest]
        for i in range(len(valid) - 1):
            p1 = project(latest[valid[i]][0]     - wrist_pos, pc, flip_x, flip_y)
            p2 = project(latest[valid[i + 1]][0] - wrist_pos, pc, flip_x, flip_y)
            cv2.line(img, p1, p2, color, 2)
            cv2.circle(img, p2, 4, (180, 180, 180), -1)

    cv2.circle(img, project([0, 0, 0], pc), 6, (255, 255, 255), -1)


def main():
    avp = AVP(ip="172.20.10.5")

    cv2.namedWindow("Hand Skeleton", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Hand Skeleton", W, H)

    pc_l = (W // 4,     H // 2)
    pc_r = (3 * W // 4, H // 2)

    while True:
        avp.get_frame()
        latest = avp.latest

        img = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.line(img, (W // 2, 0), (W // 2, H), (60, 60, 60), 1)
        cv2.putText(img, "LEFT",  (10,          30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
        cv2.putText(img, "RIGHT", (W // 2 + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
        cv2.putText(img, "q = quit", (W - 100, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 80), 1)

        draw_hand(img, latest, LEFT_CHAINS,  "wrist_l", ( 50, 200,  80), pc_l, flip_x=True, flip_y=True)
        draw_hand(img, latest, RIGHT_CHAINS, "wrist_r", ( 50, 140, 220), pc_r, flip_x=True)

        cv2.imshow("Hand Skeleton", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
