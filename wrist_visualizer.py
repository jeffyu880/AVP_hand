import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from avp_readout import AVP

W, H      = 1280, 640
AXIS_LEN  = 160   # pixels

# Fixed isometric view matrix (elev=25°, azim=35°)
_az = np.radians(35)
_el = np.radians(25)
_Ry = np.array([[np.cos(_az), 0, np.sin(_az)],
                [0, 1, 0],
                [-np.sin(_az), 0, np.cos(_az)]])
_Rx = np.array([[1, 0, 0],
                [0,  np.cos(_el), np.sin(_el)],
                [0, -np.sin(_el), np.cos(_el)]])
VIEW = _Rx @ _Ry


def project(v, pc):
    r = VIEW @ np.array(v, dtype=float)
    return (int(pc[0] + r[0] * AXIS_LEN),
            int(pc[1] - r[1] * AXIS_LEN))


def draw_frame(img, rot_m, pc):
    # X=red (roll), Y=green (yaw), Z=blue (pitch)
    axes   = [(0, 0, 255), (0, 255, 0), (255,   0, 0)]   # BGR
    labels = ["X  roll",   "Y  yaw",   "Z  pitch"]
    origin = project([0, 0, 0], pc)
    for i, (bgr, lbl) in enumerate(zip(axes, labels)):
        tip = project(rot_m[:, i], pc)
        cv2.arrowedLine(img, origin, tip, bgr, 2, tipLength=0.1)
        cv2.putText(img, lbl, (tip[0] + 6, tip[1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, bgr, 2)


def draw_reference(img, pc):
    """World axes in dim colour for reference."""
    dim = [(0, 0, 60), (0, 60, 0), (60, 0, 0)]
    origin = project([0, 0, 0], pc)
    for i, bgr in enumerate(dim):
        v = [0, 0, 0]; v[i] = 1.0
        cv2.line(img, origin, project(v, pc), bgr, 1)


def angle_text(img, rot_m, x, y):
    pitch, yaw, roll = R.from_matrix(rot_m).as_euler("zyx", degrees=True)
    cv2.putText(img, f"pitch (Z): {pitch:+6.1f} deg", (x, y),      cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200,  80,  80), 2)
    cv2.putText(img, f"yaw   (Y): {yaw:+6.1f} deg",   (x, y + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, ( 80, 200,  80), 2)
    cv2.putText(img, f"roll  (X): {roll:+6.1f} deg",  (x, y + 64), cv2.FONT_HERSHEY_SIMPLEX, 0.75, ( 80,  80, 200), 2)


def main():
    avp = AVP(ip="172.20.10.5")

    cv2.namedWindow("Wrist Rotation", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Wrist Rotation", W, H)

    pc_l = (W // 4,     H // 2)
    pc_r = (3 * W // 4, H // 2)

    while True:
        avp.get_frame()
        latest = avp.latest

        img = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.line(img, (W // 2, 0), (W // 2, H), (60, 60, 60), 1)

        cv2.putText(img, "LEFT WRIST",  (10,          30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
        cv2.putText(img, "RIGHT WRIST", (W // 2 + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
        cv2.putText(img, "X=red(roll)  Y=green(yaw)  Z=blue(pitch)    q=quit",
                    (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (90, 90, 90), 2)

        if "wrist_l" in latest:
            _, _, rot_l = latest["wrist_l"]
            draw_reference(img, pc_l)
            draw_frame(img, rot_l, pc_l)
            angle_text(img, rot_l, 10, 50)

        if "wrist_r" in latest:
            _, _, rot_r = latest["wrist_r"]
            draw_reference(img, pc_r)
            draw_frame(img, rot_r, pc_r)
            angle_text(img, rot_r, W // 2 + 10, 50)

        cv2.imshow("Wrist Rotation", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
