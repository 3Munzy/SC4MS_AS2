from camera_core import CameraCore
from target_aruco import TargetAruco
import cv2, time

cam = CameraCore(); cam.start()
det = TargetAruco()
try:
    while True:
        img, depth_mm, K, ts = cam.get_frame()
        u,v,Z,yaw,ok = det.detect(img, K, depth_mm)
        vis = img.copy()
        if ok:
            cv2.circle(vis, (int(u),int(v)), 8, (0,255,0), -1)
            if Z is not None:
                cv2.putText(vis, f"Z={Z:.3f}m yaw={0.0 if yaw is None else round(yaw,3)}",
                            (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.imshow("Aruco live (ESC to quit)", vis)
        if cv2.waitKey(1) == 27: break
finally:
    cam.stop(); cv2.destroyAllWindows()
