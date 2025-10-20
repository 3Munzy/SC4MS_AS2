from ibvs import ibvs_control
import numpy as np, time, math

K = np.array([[920,0,424],[0,920,240],[0,0,1]], float)
t0 = time.time()
print("Printing v_c every 0.1s (Ctrl-C to stop)")
try:
    while True:
        t = time.time()-t0
        u = 424 + 40*math.sin(0.5*t)
        v = 240 + 30*math.cos(0.6*t)
        Z = 0.60 + 0.03*math.sin(0.3*t)
        v_c = ibvs_control(u, v, Z, K, yaw=0.05*math.sin(t))
        print("v_c:", [round(x,4) for x in v_c])
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
