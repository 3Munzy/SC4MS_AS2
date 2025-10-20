# camera_core.py
# D435i capture on macOS (pyrealsense2-macosx)
# Provides: CameraCore.get_frame() -> (img_bgr, depth_mm, K, tstamp)

from dataclasses import dataclass
from typing import Tuple, Optional
import time
import numpy as np
import pyrealsense2 as rs
import cv2


@dataclass
class CameraConfig:
    width: int = 848
    height: int = 480
    fps: int = 60
    align_to: rs.stream = rs.stream.color  # align depth->color


class CameraCore:
    def __init__(self, cfg: CameraConfig = CameraConfig()):
        self.cfg = cfg
        self._pipe: Optional[rs.pipeline] = None
        self._align: Optional[rs.align] = None
        self._K: Optional[np.ndarray] = None
        self._started = False

    def start(self):
        if self._started:
            return
        self._pipe = rs.pipeline()
        config = rs.config()
        # Enable both streams at same resolution/FPS
        config.enable_stream(rs.stream.color, self.cfg.width, self.cfg.height, rs.format.bgr8, self.cfg.fps)
        config.enable_stream(rs.stream.depth, self.cfg.width, self.cfg.height, rs.format.z16, self.cfg.fps)
        prof = self._pipe.start(config)
        self._align = rs.align(self.cfg.align_to)

        # Intrinsics (from color stream)
        color_stream = prof.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        self._K = np.array([
            [intr.fx,   0.0,    intr.ppx],
            [0.0,       intr.fy, intr.ppy],
            [0.0,       0.0,    1.0],
        ], dtype=float)

        # Small warm-up for auto-exposure
        for _ in range(10):
            self._pipe.wait_for_frames()

        self._started = True

    def stop(self):
        if self._pipe is not None:
            try:
                self._pipe.stop()
            except Exception:
                pass
        self._pipe = None
        self._align = None
        self._started = False

    @property
    def K(self) -> np.ndarray:
        if self._K is None:
            raise RuntimeError("Camera not started; call start() first")
        return self._K

    def get_frame(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """
        Returns:
            img_bgr:  (H,W,3) uint8
            depth_mm: (H,W)   uint16 (millimetres)
            K:        (3,3)   float64 camera intrinsics
            tstamp:   float   time.time() seconds
        """
        if not self._started:
            self.start()

        frames = self._pipe.wait_for_frames()
        frames = self._align.process(frames)

        cf = frames.get_color_frame()
        df = frames.get_depth_frame()
        if not cf or not df:
            raise RuntimeError("Failed to read color/depth frames")

        img_bgr = np.asanyarray(cf.get_data())            # uint8
        depth_mm = np.asanyarray(df.get_data())           # uint16 millimetres
        tstamp = time.time()
        return img_bgr, depth_mm, self._K, tstamp


# ---- quick demo ----
if __name__ == "__main__":
    cam = CameraCore()
    cam.start()
    fps_t0 = time.time(); frames = 0
    try:
        while True:
            img, depth_mm, K, ts = cam.get_frame()
            # Visualize depth as preview
            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_mm, alpha=0.03), cv2.COLORMAP_JET
            )
            both = np.hstack([img, depth_vis])
            cv2.putText(both, f"fx={K[0,0]:.1f} fy={K[1,1]:.1f}",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow("D435i (color | depth)", both)
            if cv2.waitKey(1) == 27:  # ESC to quit
                break
            frames += 1
            if time.time() - fps_t0 >= 1.0:
                print(f"FPS ~ {frames}")
                frames = 0; fps_t0 = time.time()
    finally:
        cam.stop()
        cv2.destroyAllWindows()
