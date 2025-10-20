import numpy as np

def ibvs_control(u, v, Z, K, Z_des=0.55, lam_xy=0.4, lam_z=0.3, k_yaw=0.5, yaw=0):
    """
    Compute a 6D camera-frame twist for IBVS.

    Args:
        u, v: pixel coordinates of target
        Z: current depth (m)
        K: camera intrinsics (3x3)
        Z_des: desired depth
        lam_xy, lam_z, k_yaw: control gains
        yaw: tag yaw about camera Z

    Returns:
        v_c: np.array([vx, vy, vz, wx, wy, wz]) (m/s, rad/s)
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # pixel error (target - image center)
    ex = u - cx
    ey = v - cy

    # normalised pixel errors → metres (scaled by depth)
    vx = -lam_xy * (ex / fx) * Z
    vy = -lam_xy * (ey / fy) * Z
    vz = -lam_z  * (Z - Z_des)
    wz = -k_yaw  * yaw

    return np.array([vx, vy, vz, 0.0, 0.0, wz])
