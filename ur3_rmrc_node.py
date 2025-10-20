#!/usr/bin/env python3
"""
UR3e RMRC controller (ROS + Swift)
- Consumes a 6D twist (geometry_msgs/Twist) from /ibvs/twist
- Maps to joint velocities via damped least-squares J⁺
- Publishes to /scaled_joint_velocity_controller/command (Float64MultiArray)

Modes:
  SIM (default): Swift viewer, integrates qdot on the model
  ROS (USE_ROS=1): runs against real robot via rosbridge

Env vars you can set:
  USE_ROS=1                 # talk to real robot
  ROS_HOST=192.168.27.1     # rosbridge host
  ROS_PORT=9090             # rosbridge port
  TWIST_FRAME=base|tool     # input twist frame (default 'base')
  RATE_HZ=60                # control loop rate
"""

import os, time, math
import numpy as np

USE_ROS   = os.getenv("USE_ROS", "0") == "1"
ROS_HOST  = os.getenv("ROS_HOST", "192.168.27.1")
ROS_PORT  = int(os.getenv("ROS_PORT", "9090"))
TWIST_FRAME = os.getenv("TWIST_FRAME", "base").lower()  # 'base' or 'tool'
RATE_HZ  = float(os.getenv("RATE_HZ", "60"))

# Safety + control params
LAMBDA = 1e-4                                 # damping for J+
QDOT_MAX = np.array([1.4, 1.4, 1.7, 2.0, 2.2, 2.2])  # rad/s per joint (UR3 soft caps)
LPF_ALPHA = 0.35                              # joint-vel smoothing [0..1]
WARMUP_FRAMES = 8                             # require N good twist msgs before motion
WATCHDOG_S = 0.6                              # if twist stale -> zero

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

# ---------------- Utilities ----------------
def clamp_vec(v, vmax):
    s = np.sign(v)
    return s * np.minimum(np.abs(v), vmax)

def lpf(prev, new, alpha=LPF_ALPHA):
    return alpha*new + (1.0-alpha)*prev

def skew(p):
    px, py, pz = p
    return np.array([[0, -pz,  py],
                     [pz,  0, -px],
                     [-py, px,  0]], dtype=float)

def adjoint_SE3(T):
    """Adjoint matrix for SE3 to map twists: v_out = Ad(T) @ v_in"""
    R = np.array(T.R)
    p = np.array(T.t).reshape(3)
    Ad = np.zeros((6,6))
    Ad[0:3,0:3] = R
    Ad[3:6,3:6] = R
    Ad[0:3,3:6] = skew(p) @ R
    return Ad

# ---------------- RMRC core ----------------
class RMRCController:
    def __init__(self):
        import roboticstoolbox as rtb
        self.r = rtb.models.URDF.UR3()   # RTB kinematic model (DH/URDF)
        self.v0 = np.zeros(6)            # desired base-frame twist
        self.have_twist = False
        self.last_twist_t = 0.0
        self._qdot_prev = np.zeros(6)
        self._warm = 0

    def set_twist_base(self, v0):
        """Set desired base-frame twist ^0V (6,)"""
        self.v0 = np.asarray(v0, dtype=float)
        self.have_twist = True
        self.last_twist_t = time.time()

    def set_twist_tool(self, ve):
        """Set desired tool-frame twist ^eV (6,), converts to base using current FK"""
        from spatialmath import SE3
        ve = np.asarray(ve, dtype=float)
        T0e = self.r.fkine(self.r.q)      # ^0T_e
        Ad_0e = adjoint_SE3(T0e)
        v0 = Ad_0e @ ve
        self.set_twist_base(v0)

    def step(self, q, dt):
        """
        One control step:
        - If twist fresh and warm-up OK: compute qdot = J0+ * v0
        - Apply clamps + smoothing
        - Returns qdot (for ROS) and q_next (for SIM integration)
        """
        # Watchdog
        if (time.time() - self.last_twist_t) > WATCHDOG_S:
            self.have_twist = False
            self._warm = 0

        # No valid twist yet -> zero
        if not self.have_twist:
            self._qdot_prev = np.zeros(6)
            return np.zeros(6), q

        # Warm-up
        self._warm += 1
        if self._warm < WARMUP_FRAMES:
            self._qdot_prev = np.zeros(6)
            return np.zeros(6), q

        # Compute Jacobian and map
        J = self.r.jacob0(q)                     # 6x6 geometric Jacobian in base frame
        JT = J.T
        JJt = J @ JT
        qdot = JT @ np.linalg.inv(JJt + LAMBDA*np.eye(6)) @ self.v0

        # Safety: clamp & smooth
        qdot = clamp_vec(qdot, QDOT_MAX)
        qdot_s = lpf(self._qdot_prev, qdot, LPF_ALPHA)
        self._qdot_prev = qdot_s

        # For SIM: integrate q
        q_next = q + qdot_s * dt
        return qdot_s, q_next

# ---------------- ROS mode ----------------
def run_ros():
    import roslibpy
    from spatialmath import SE3  # for possible tool-frame option

    ctrl = RMRCController()
    client = roslibpy.Ros(ROS_HOST, ROS_PORT)
    client.run()
    if not client.is_connected:
        raise RuntimeError(f"Cannot connect to rosbridge at {ROS_HOST}:{ROS_PORT}")
    print("[ROS] Connected to rosbridge.")

    # Subscribers
    q_holder = {'q': None}
    def on_js(msg):
        # Prefer standard /joint_states, but keep your custom if needed
        names = msg.get('name', [])
        pos = msg.get('position', [])
        if len(pos) >= 6:
            # Map onto expected UR3 joint order if names provided
            order = []
            if names and all(n in names for n in JOINT_NAMES):
                idx = [names.index(n) for n in JOINT_NAMES]
                order = [pos[i] for i in idx]
            else:
                order = pos[:6]
            q_holder['q'] = np.array(order, dtype=float)

    # Try both common topics; subscribe to whichever you have
    sub_js = roslibpy.Topic(client, '/joint_states', 'sensor_msgs/JointState')
    sub_js.subscribe(on_js)
    sub_js2 = roslibpy.Topic(client, '/ur/joint_states', 'sensor_msgs/JointState')
    sub_js2.subscribe(on_js)

    def on_twist(msg):
        lin = msg.get('linear', {})
        ang = msg.get('angular', {})
        v = np.array([lin.get('x',0.0), lin.get('y',0.0), lin.get('z',0.0),
                      ang.get('x',0.0), ang.get('y',0.0), ang.get('z',0.0)], dtype=float)
        if TWIST_FRAME == 'tool':
            ctrl.set_twist_tool(v)
        else:
            ctrl.set_twist_base(v)

    sub_twist = roslibpy.Topic(client, '/ibvs/twist', 'geometry_msgs/Twist')
    sub_twist.subscribe(on_twist)

    # Publisher (joint velocities)
    pub_jvel = roslibpy.Topic(client, '/scaled_joint_velocity_controller/command', 'std_msgs/Float64MultiArray')

    # Loop
    dt = 1.0 / RATE_HZ
    try:
        print("[ROS] Waiting for /joint_states and /ibvs/twist ...")
        while client.is_connected:
            if q_holder['q'] is None:
                time.sleep(0.01); continue

            qdot, _ = ctrl.step(q_holder['q'], dt)
            pub_jvel.publish(roslibpy.Message({'data': qdot.tolist()}))
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[ROS] Ctrl-C, stopping.")
    finally:
        # soft stop
        try: pub_jvel.publish(roslibpy.Message({'data':[0.0]*6}))
        except: pass
        sub_twist.unsubscribe()
        sub_js.unsubscribe(); sub_js2.unsubscribe()
        client.terminate()
        print("[ROS] Disconnected.")

# ---------------- SIM mode ----------------
def run_sim():
    import roboticstoolbox as rtb
    from roboticstoolbox.backends.swift import Swift

    ctrl = RMRCController()
    env = Swift()
    env.launch(realtime=True)

    r = ctrl.r
    env.add(r)

    # Simple demo input if no external twist is provided:
    # publish a slow tool-frame sway for sanity.
    demo = True
    t0 = time.time()

    dt = 1.0 / RATE_HZ
    try:
        print("[SIM] Running. Close the viewer or Ctrl-C to quit.")
        while True:
            if demo:
                t = time.time() - t0
                # small circular motion demand in base frame (m/s)
                vx = 0.02 * math.sin(2*math.pi*0.1*t)
                vy = 0.02 * math.cos(2*math.pi*0.1*t)
                v0 = np.array([vx, vy, 0.0, 0.0, 0.0, 0.0])
                ctrl.set_twist_base(v0)

            qdot, q_next = ctrl.step(r.q, dt)
            r.q = q_next
            env.step()
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[SIM] Stopping.")
    finally:
        env.hold()

# ---------------- Entrypoint ----------------
if __name__ == "__main__":
    if USE_ROS:
        run_ros()
    else:
        run_sim()

