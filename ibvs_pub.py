# ibvs_pub.py (Mac)
import os, time
import roslibpy
import numpy as np
from camera_core import CameraCore
from target_aruco import TargetAruco
from ibvs import ibvs_control   # your IBVS math

ROS_HOST = os.getenv("ROS_HOST", "192.168.27.1")
ROS_PORT = int(os.getenv("ROS_PORT", "9090"))

def main():
    # 1) Start camera locally (no ROS)
    cam = CameraCore(); cam.start()
    det = TargetAruco()

    # 2) Open rosbridge to the Pi
    client = roslibpy.Ros(ROS_HOST, ROS_PORT)
    client.run()
    pub = roslibpy.Topic(client, '/ibvs/twist', 'geometry_msgs/Twist')
    pub.advertise()
    print("Connected to rosbridge; publishing /ibvs/twist")

    warm = 0
    try:
        while client.is_connected:
            img, depth_mm, K, ts = cam.get_frame()
            u,v,Z,yaw,ok = det.detect(img, K, depth_mm)
            if ok and Z is not None:
                warm += 1
                if warm >= 8:
                    v_c = ibvs_control(u, v, Z, K, yaw=yaw)  # camera/tool-frame twist
                    msg = {'linear': {'x': float(v_c[0]), 'y': float(v_c[1]), 'z': float(v_c[2])},
                           'angular': {'x': float(v_c[3]), 'y': float(v_c[4]), 'z': float(v_c[5])}}
                    pub.publish(roslibpy.Message(msg))
            else:
                warm = 0
                pub.publish(roslibpy.Message({'linear':{'x':0,'y':0,'z':0},'angular':{'x':0,'y':0,'z':0}}))
    except KeyboardInterrupt:
        pass
    finally:
        pub.unadvertise()
        client.terminate()
        cam.stop()
        print("Stopped.")

if __name__ == "__main__":
    main()
