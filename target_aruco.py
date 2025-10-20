# target_aruco.py
# Detect a 4x4 ArUco tag and derive (u,v), depth Z (m), and yaw.

from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np
import cv2


@dataclass
class ArucoConfig:
    marker_id: int = 69
    marker_size_m: float = 0.169   # physical side length (m)
    median_win: int = 7           # window for depth median (px, odd)
    min_depth_m: float = 0.05     # reject depths less than this (m)


class TargetAruco:
    def __init__(self, cfg: ArucoConfig = ArucoConfig()):
        self.cfg = cfg
        self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
        # support both old & newer OpenCV APIs
        try:
            self._params = cv2.aruco.DetectorParameters_create()
            self._detector = None
        except AttributeError:
            self._params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(self._dict, self._params)

    @staticmethod
    def _median_depth(depth_mm: np.ndarray, u: float, v: float, k: int) -> Optional[float]:
        h, w = depth_mm.shape[:2]
        u_i, v_i = int(round(u)), int(round(v))
        r = k // 2
        u0, u1 = max(0, u_i-r), min(w, u_i+r+1)
        v0, v1 = max(0, v_i-r), min(h, v_i+r+1)
        window = depth_mm[v0:v1, u0:u1].astype(np.float32)
        valid = window[window > 50.0]  # > 50 mm
        if valid.size == 0:
            return None
        return float(np.median(valid)) / 1000.0  # metres

    def detect(self, img_bgr: np.ndarray, K: np.ndarray, depth_mm: Optional[np.ndarray] = None
               ) -> Tuple[float, float, Optional[float], Optional[float], bool]:
        """
        Args:
            img_bgr:  (H,W,3) uint8
            K:        (3,3) intrinsics
            depth_mm: (H,W) uint16 millimetres (optional but needed for Z)

        Returns:
            u, v:         centroid in pixels
            Z_m:          depth at (u,v) in metres (or None if unavailable)
            yaw_rad:      approximate yaw of tag about camera Z (or None if not computed)
            ok:           True if target found
        """
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(img_bgr)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(img_bgr, self._dict, parameters=self._params)

        if ids is None:
            return 0.0, 0.0, None, None, False

        ids = ids.flatten()
        if self.cfg.marker_id not in ids:
            return 0.0, 0.0, None, None, False

        idx = int(np.where(ids == self.cfg.marker_id)[0][0])
        c = corners[idx].reshape(4, 2)  # (4,2)
        u = float(np.mean(c[:, 0])); v = float(np.mean(c[:, 1]))

        # Depth
        Z_m = None
        if depth_mm is not None:
            Z_m = self._median_depth(depth_mm, u, v, self.cfg.median_win)
            if Z_m is not None and Z_m < self.cfg.min_depth_m:
                Z_m = None

        # Yaw via PnP (optional)
        yaw_rad = None
        try:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[idx], self.cfg.marker_size_m, K, None
            )
            rvec = rvec.reshape(3)
            R, _ = cv2.Rodrigues(rvec)
            # yaw about camera Z
            yaw_rad = float(np.arctan2(R[1, 0], R[0, 0]))
        except Exception:
            yaw_rad = None

        return u, v, Z_m, yaw_rad, True


# ---- quick demo (press ESC to quit) ----
if __name__ == "__main__":
    # Simple webcam fallback demo; replace with camera_core for D435i live test
    print("Open a test image with an ArUco tag or integrate with camera_core.py for live feed.")
    import sys
    if len(sys.argv) < 2:
        print("usage: python target_aruco.py path/to/image.png")
        sys.exit(0)

    img = cv2.imread(sys.argv[1], cv2.IMREAD_COLOR)
    if img is None:
        print("Failed to load image")
        sys.exit(1)

    # Fake intrinsics for demo (rough 800 px focal)
    K = np.array([[800, 0, img.shape[1]/2],
                  [0, 800, img.shape[0]/2],
                  [0,   0, 1.0]], dtype=float)

    det = TargetAruco()
    u, v, Z, yaw, ok = det.detect(img, K, None)
    print("ok:", ok, "u,v:", (u,v), "Z:", Z, "yaw:", yaw)
    if ok:
        cv2.circle(img, (int(u), int(v)), 6, (0, 255, 0), -1)
    cv2.imshow("Aruco demo", img)
    cv2.waitKey(0)
