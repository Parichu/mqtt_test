"""
calibrate_camera_to_map.py
==========================
รันครั้งเดียวเพื่อ calibrate กล้อง → แผนที่ SLAM

Dependencies: pip install opencv-python numpy pyyaml
"""

import cv2
import numpy as np
import json
import yaml
import os

# ============================================================
# CONFIG — แก้ตรงนี้
# ============================================================
CAMERA_SOURCE = 2
MAP_YAML_PATH = "/home/parichu/ros2_ws/mqtt_test/src/mqtt_test/config/mapuse28.yaml"
HOMOGRAPHY_OUT = "homography.json"

# หมุนภาพกล้อง (แก้ให้กล้องหันตรงขึ้น ก่อน calibrate)
# ลอง 1 ค่าแล้ว reset calibrate ใหม่ ดูว่าห้องตรงกับแผนที่ไหม
# None                        = ไม่หมุน
# cv2.ROTATE_90_CLOCKWISE     = หมุนขวา 90°
# cv2.ROTATE_90_COUNTERCLOCKWISE = หมุนซ้าย 90°  ← ลองอันนี้ก่อน (กล้องเอียงซ้าย 45° → อาจต้องหมุน)
# cv2.ROTATE_180              = กลับหัว
CAM_ROTATE = None


# ============================================================
# โหลด map.yaml และ map.pgm
# ============================================================
def load_map(yaml_path: str):
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    img_field = cfg["image"]
    if os.path.isabs(img_field):
        pgm_path = img_field
    else:
        pgm_path = os.path.join(os.path.dirname(yaml_path), img_field)
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"ไม่พบ {pgm_path}")
    cfg["height_px"] = img.shape[0]
    return img, cfg


# ============================================================
# State
# ============================================================
class CalibState:
    def __init__(self):
        self.cam_pts = []
        self.map_pts = []
        self.H = None
        self.pending = None
        self.mode = "idle"


state = CalibState()
cam_disp = None
map_disp = None
map_base = None
map_gray_g = None  # เก็บ grayscale ไว้ใช้ใน visualizer


def rotate_frame(frame):
    if CAM_ROTATE is None:
        return frame
    return cv2.rotate(frame, CAM_ROTATE)


# ============================================================
# Mouse callbacks
# ============================================================
def on_cam_click(event, u, v, flags, _):
    if event != cv2.EVENT_LBUTTONDOWN or state.mode != "cam":
        return
    state.pending = (u, v)
    state.mode = "map"
    print(f"  [กล้อง] ({u},{v})  →  ตอนนี้คลิกจุดเดียวกันบน Map")


def on_map_click(event, col, row, flags, _):
    global map_disp
    if event != cv2.EVENT_LBUTTONDOWN or state.mode != "map":
        return
    if state.pending is None:
        return
    state.cam_pts.append(state.pending)
    state.map_pts.append((col, row))
    n = len(state.cam_pts)
    print(f"  [แผนที่] ({col},{row})  →  บันทึกคู่ที่ {n}")
    cv2.circle(map_disp, (col, row), 7, (0, 200, 0), -1)
    cv2.putText(
        map_disp,
        str(n),
        (col + 9, row + 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 200, 0),
        1,
    )
    state.pending = None
    state.mode = "cam"
    print(f"  คู่ถัดไป — คลิกบนกล้องได้เลย (มี {n} คู่แล้ว)")


# ============================================================
# Homography
# ============================================================
def compute_H():
    if len(state.cam_pts) < 4:
        print("[!] ต้องการอย่างน้อย 4 คู่")
        return False
    src = np.float32(state.cam_pts)
    dst = np.float32(state.map_pts)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    if H is None:
        print("[!] คำนวณ H ไม่ได้ — ลองเพิ่มจุดหรือเลือกจุดใหม่")
        return False
    inliers = int(mask.sum())
    state.H = H
    errs = []
    for (u, v), (c, r) in zip(state.cam_pts, state.map_pts):
        pt = np.float32([[[u, v]]])
        res = cv2.perspectiveTransform(pt, H)[0][0]
        errs.append(float(np.linalg.norm(res - np.array([c, r]))))
    mean_err = np.mean(errs)
    max_err = max(errs)
    print(f"[H] คำนวณสำเร็จ — {inliers}/{len(src)} inliers")
    print(f"    reprojection error: {mean_err:.2f} px (mean)  {max_err:.2f} px (max)")
    if mean_err > 10:
        print("    [!] error สูงเกินไป — ลองเลือกจุดใหม่ให้ตรงกว่านี้")
    else:
        print("    [OK] error อยู่ในเกณฑ์ดี")
    return True


def visualize_reprojection():
    """กด V หลัง H — แสดงจุดจริง (เขียว) vs ที่ H ทำนาย (แดง) บนแผนที่"""
    if state.H is None or map_gray_g is None:
        print("[!] คำนวณ H ก่อน")
        return
    vis = cv2.cvtColor(map_gray_g.copy(), cv2.COLOR_GRAY2BGR)
    for i, ((u, v), (c, r)) in enumerate(zip(state.cam_pts, state.map_pts)):
        cv2.circle(vis, (c, r), 7, (0, 200, 0), -1)  # จริง = เขียว
        pt = np.float32([[[u, v]]])
        res = cv2.perspectiveTransform(pt, state.H)[0][0]
        pc, pr = int(res[0]), int(res[1])
        cv2.circle(vis, (pc, pr), 7, (0, 0, 255), -1)  # ทำนาย = แดง
        cv2.line(vis, (c, r), (pc, pr), (255, 150, 0), 1)
        cv2.putText(
            vis, str(i + 1), (c + 9, r), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1
        )
    cv2.imshow("Reprojection  green=actual  red=predicted", vis)
    cv2.waitKey(0)
    cv2.destroyWindow("Reprojection  green=actual  red=predicted")


def save_H():
    if state.H is None:
        print("[!] กด H ก่อน")
        return
    with open(MAP_YAML_PATH) as f:
        cfg = yaml.safe_load(f)
    data = {
        "H": state.H.tolist(),
        "cam_rotate": str(CAM_ROTATE),
        "cam_points": state.cam_pts,
        "map_points": state.map_pts,
        "map_yaml": {
            "resolution": cfg["resolution"],
            "origin": cfg["origin"],
            "image": cfg["image"],
            "height_px": map_gray_g.shape[0] if map_gray_g is not None else 0,
        },
    }
    with open(HOMOGRAPHY_OUT, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[บันทึก] {HOMOGRAPHY_OUT}")


# ============================================================
# HUD
# ============================================================
def draw_hud(frame, extra=""):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, h - 52), (w, h), (0, 0, 0), -1)
    rot_label = {
        str(cv2.ROTATE_90_CLOCKWISE): "rot=CW90",
        str(cv2.ROTATE_90_COUNTERCLOCKWISE): "rot=CCW90",
        str(cv2.ROTATE_180): "rot=180",
        "None": "rot=none",
    }.get(str(CAM_ROTATE), "rot=?")
    line1 = (
        f"Mode:{state.mode.upper()}  Pairs:{len(state.cam_pts)}"
        f"  H:{'OK' if state.H is not None else '-'}  {rot_label}"
    )
    line2 = "C=เริ่ม  H=คำนวณ  V=visualize  S=บันทึก  R=reset  Q=ออก"
    if extra:
        line2 += f"  | {extra}"
    cv2.putText(
        frame, line1, (8, h - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1
    )
    cv2.putText(
        frame, line2, (8, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1
    )


# ============================================================
# Main
# ============================================================
def main():
    global cam_disp, map_disp, map_base, map_gray_g

    if not os.path.exists(MAP_YAML_PATH):
        print(f"[!] ไม่พบ {MAP_YAML_PATH}")
        return

    map_gray, map_cfg = load_map(MAP_YAML_PATH)
    map_gray_g = map_gray
    map_base = cv2.cvtColor(map_gray, cv2.COLOR_GRAY2BGR)
    map_disp = map_base.copy()

    print(
        f"[map.yaml] resolution={map_cfg['resolution']} m/px  "
        f"origin={map_cfg['origin']}  "
        f"size={map_gray.shape[1]}×{map_gray.shape[0]} px"
    )
    print(
        f"[กล้อง] หมุนภาพ: {CAM_ROTATE}  (แก้ CAM_ROTATE ใน CONFIG ถ้าห้องยังไม่ตรงกับแผนที่)\n"
    )

    cap = cv2.VideoCapture(CAMERA_SOURCE)
    if not cap.isOpened():
        print(f"[!] เปิดกล้องไม่ได้: {CAMERA_SOURCE}")
        return

    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Map (pgm)", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Camera", on_cam_click)
    cv2.setMouseCallback("Map (pgm)", on_map_click)

    print("=== Calibration Tool ===")
    print("1. ดูหน้าต่างกล้อง — ตรวจว่าทิศห้องตรงกับแผนที่ไหม")
    print("   ถ้าไม่ตรง แก้ CAM_ROTATE แล้วรันใหม่")
    print("2. กด C → คลิกกล้อง → คลิกแผนที่ (≥8 คู่ กระจายทั่วห้อง)")
    print("3. กด H → V (ดู error) → S (บันทึก)\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = rotate_frame(frame)
        cam_disp = frame.copy()

        for i, (u, v) in enumerate(state.cam_pts):
            cv2.circle(cam_disp, (u, v), 5, (0, 255, 0), -1)
            cv2.putText(
                cam_disp,
                str(i + 1),
                (u + 7, v + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
            )
        if state.pending:
            cv2.circle(cam_disp, state.pending, 9, (0, 165, 255), 2)

        hint = (
            "คลิกบนกล้อง"
            if state.mode == "cam"
            else "คลิกบนแผนที่"
            if state.mode == "map"
            else ""
        )
        draw_hud(cam_disp, hint)
        cv2.imshow("Camera", cam_disp)
        cv2.imshow("Map (pgm)", map_disp)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("c"):
            state.mode = "cam"
            print("[เริ่ม] คลิกบนกล้อง")
        elif key == ord("h"):
            compute_H()
        elif key == ord("v"):
            visualize_reprojection()
        elif key == ord("s"):
            save_H()
        elif key == ord("r"):
            state.cam_pts.clear()
            state.map_pts.clear()
            state.H = None
            state.mode = "idle"
            map_disp = map_base.copy()
            print("[Reset] ล้างข้อมูลทั้งหมด")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
