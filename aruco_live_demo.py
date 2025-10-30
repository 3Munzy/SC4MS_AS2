# aruco_live_ibvs_demo.py
# Live ArUco → IBVS: compute camera-frame velocity needed to center the marker.
# - Translation (vx, vy, vz) drives the CAMERA toward the marker center (flipped vs feature-centering).
# - Yaw correction (wz) keeps the original sign (not flipped).
#
# Requirements: opencv-contrib-python, numpy
# Your project files: camera_core.py, target_aruco.py

import time, math
import numpy as np
import cv2

# ---- CONFIG ----
# Desired standoff distance (m)
Z_DES      = 0.55
# IBVS gains
LAM_XY     = 0.40   # pixel-plane regulation gain
LAM_Z      = 0.30   # approach gain
K_YAW      = 0.50   # yaw stabilisation gain (rad/s per rad)
# Safety/limits
Z_MIN      = 0.10   # don't use depth below this
V_LIN_MAX  = 0.25   # cap |[vx,vy,vz]|
WZ_MAX     = np.deg2rad(60.0)
# Drawing scale for on-frame velocity arrows (pixels per m/s)
DRAW_SCALE = 220.0

# ---- UTILITIES ----
def ibvs_twist_camera(u, v, Z, K, yaw):
    """
    Compute CAMERA-frame twist to move the *camera* so the marker centers in the image.

    Inputs:
      u,v : marker pixel center
      Z   : marker range (m)
      K   : 3x3 intrinsics
      yaw : marker yaw about camera Z (rad), +ve = CCW

    Returns:
      np.array([vx, vy, vz, wx, wy, wz])
    """
    Z_use = max(float(Z), Z_MIN) if Z is not None else Z_DES

    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    ex = (u - cx) / fx
    ey = (v - cy) / fy

    # ✅ Flip TRANSLATION so the CAMERA moves to center the marker
    vx = +LAM_XY * ex * Z_use
    vy = +LAM_XY * ey * Z_use
    vz = +LAM_Z  * (Z_use - Z_DES)

    # ✅ Keep YAW sign as original (no flip)
    wx = 0.0
    wy = 0.0
    wz = -K_YAW * (yaw if yaw is not None else 0.0)

    # clamp linear speed magnitude
    vlin = math.sqrt(vx*vx + vy*vy + vz*vz)
    if vlin > V_LIN_MAX:
        s = V_LIN_MAX / vlin
        vx, vy, vz = vx*s, vy*s, vz*s
    # clamp yaw rate
    wz = float(np.clip(wz, -WZ_MAX, WZ_MAX))

    return np.array([vx, vy, vz, wx, wy, wz], float)

def put_text(img, text, org, color=(255,255,255), scale=0.6, thick=2):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

def draw_velocity_overlay(vis, u, v, K, twist):
    """
    Draw arrow(s) indicating commanded camera translation projected in pixel space.
    We visualize only vx, vy (image-plane lateral) for clarity.
    """
    vx, vy, vz, _, _, wz = twist
    h, w = vis.shape[:2]
    cx, cy = int(K[0,2]), int(K[1,2])

    # Draw error arrow (pixel error from center to detection)
    cv2.arrowedLine(vis, (cx, cy), (int(u), int(v)), (0, 200, 255), 2, tipLength=0.25)

    # Draw commanded lateral camera motion as an arrow starting from the image center
    # Map (vx, vy) [m/s] → pixels via a fixed scale for visualization
    end = (int(cx + DRAW_SCALE*vx), int(cy + DRAW_SCALE*vy))
    cv2.arrowedLine(vis, (cx, cy), end, (0, 255, 0), 3, tipLength=0.25)

    # HUD text
    put_text(vis, f"cmd v: [{vx:+.3f}, {vy:+.3f}, {vz:+.3f}] m/s", (10, 30), (0,255,0))
    put_text(vis, f"cmd w: [0.000, 0.000, {wz:+.3f}] rad/s",     (10, 55), (0,255,0))
    put_text(vis, "arrows:  teal = pixel error,  green = camera motion cmd", (10, 80), (200,255,200), 0.55, 1)

# ---- MAIN ----
def main():
    # Lazy import of your camera + detector so this file stays self-contained
    from camera_core import CameraCore
    from target_aruco import TargetAruco, ArucoConfig

    print("ArUco IBVS demo: press ENTER to start camera.")
    input()

    cam = CameraCore()
    cam.start()
    det = TargetAruco(ArucoConfig())  # your defaults (DICT_7X7_100, marker_id, etc)
    print(f"[i] Searching for ArUco id={det.cfg.marker_id}")

    # Priming frame so window appears quickly
    vis0 = np.zeros((220, 420, 3), np.uint8)
    vis0[:] = (40, 60, 80)
    put_text(vis0, "Starting camera...", (14, 120))
    cv2.imshow("ArUco IBVS (ESC to quit)", vis0)
    cv2.waitKey(1)

    t_prev = time.time()
    fps_cnt, fps_t0 = 0, time.time()

    try:
        while True:
            img, depth_mm, K, ts = cam.get_frame()
            vis = img.copy()
            u, v, Z, yaw, ok = det.detect(img, K, depth_mm)

            if ok:
                # Compute twist and print/log
                twist = ibvs_twist_camera(u, v, Z, K, yaw)
                vx, vy, vz, wx, wy, wz = twist

                # Console print (replace with your control pipe if needed)
                print(f"cmd_twist: vx={vx:+.3f} vy={vy:+.3f} vz={vz:+.3f} | wz={wz:+.3f}  (Z={Z:.3f} yaw={yaw:+.3f})")

                # Visual overlays
                cv2.circle(vis, (int(u), int(v)), 8, (0,255,0), -1)
                draw_velocity_overlay(vis, u, v, K, twist)
            else:
                put_text(vis, "No ArUco detected — check dict/ID/lighting/focus.", (10, 30), (0,255,255))

            # FPS
            fps_cnt += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                put_text(vis, f"FPS: {fps_cnt}", (10, 105))
                fps_cnt, fps_t0 = 0, now

            cv2.imshow("ArUco IBVS (ESC to quit)", vis)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
