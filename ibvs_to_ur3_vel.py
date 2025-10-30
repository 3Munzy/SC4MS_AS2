#!/usr/bin/env python3
"""
ibvs_to_ur3_vel.py
ROS 2 node: convert camera-frame twist (IBVS) -> UR3 joint velocities via DLS inverse kinematics.

I/O
- Sub:  /ibvs/camera_twist (geometry_msgs/TwistStamped)  # ^cV (camera frame)
- Sub:  /joint_states      (sensor_msgs/JointState)
- Pub:  /joint_group_velocity_controller/commands (std_msgs/Float64MultiArray)  # qdot [rad/s]

Notes
- Uses roboticstoolbox UR3 kinematics (no Gazebo meshes needed).
- Applies ^eT_c (hand–eye) and ^0T_e(q) adjoints to get ^0V, then DLS to get qdot.
- Velocity clamped per-joint and lightly low-pass filtered for smoothness.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import roboticstoolbox as rtb
from spatialmath import SE3

# ---------- small helpers ----------
def adjoint_SE3(T: SE3) -> np.ndarray:
    """Adjoint matrix Ad(T) for SE3 (6x6)."""
    R = T.R
    p = T.t.reshape(3, 1)
    px = np.array([
        [0,        -p[2, 0],  p[1, 0]],
        [p[2, 0],   0,       -p[0, 0]],
        [-p[1, 0],  p[0, 0],  0]
    ])
    Ad = np.zeros((6, 6))
    Ad[0:3, 0:3] = R
    Ad[3:6, 3:6] = R
    Ad[0:3, 3:6] = px @ R
    return Ad


class IBVStoUR3Vel(Node):
    def __init__(self):
        super().__init__('ibvs_to_ur3_vel')

        # --- Parameters (declare + get) ---
        # Hand–eye: camera pose in tool frame ^eT_c (meters, degrees)
        self.declare_parameter('handeye_tx', 0.08)   # camera +X forward from tool [m]
        self.declare_parameter('handeye_ty', 0.0)
        self.declare_parameter('handeye_tz', 0.0)
        self.declare_parameter('handeye_rx_deg', 0.0)  # extrinsics small tilt if needed
        self.declare_parameter('handeye_ry_deg', -2.0)
        self.declare_parameter('handeye_rz_deg', 0.0)

        # Damped least squares lambda
        self.declare_parameter('dls_lambda', 0.02)

        # Joint velocity limits [deg/s] -> converted to rad/s
        self.declare_parameter('joint_vel_limit_deg', 30.0)

        # Low-pass smoothing (0..1), higher = more weight on new qdot
        self.declare_parameter('lpf_alpha', 0.3)

        # Publish rate (Hz)
        self.declare_parameter('rate_hz', 100.0)

        # Controller topic (change if your Gazebo controller differs)
        self.declare_parameter('cmd_topic', '/forward_velocity_controller/commands')

        # Fetch params
        tx = float(self.get_parameter('handeye_tx').value)
        ty = float(self.get_parameter('handeye_ty').value)
        tz = float(self.get_parameter('handeye_tz').value)
        rx = math.radians(float(self.get_parameter('handeye_rx_deg').value))
        ry = math.radians(float(self.get_parameter('handeye_ry_deg').value))
        rz = math.radians(float(self.get_parameter('handeye_rz_deg').value))

        self.LAMBDA = float(self.get_parameter('dls_lambda').value)
        self.QDOT_MAX = math.radians(float(self.get_parameter('joint_vel_limit_deg').value))
        self.LPF_ALPHA = float(self.get_parameter('lpf_alpha').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)

        # Hand–eye transform ^eT_c
        self.T_e_c = SE3.Tx(tx) @ SE3.Ty(ty) @ SE3.Tz(tz) @ SE3.Rx(rx) @ SE3.Ry(ry) @ SE3.Rz(rz)
        self.Ad_e_c = adjoint_SE3(self.T_e_c)

        # --- Robot model (UR3 kinematics only) ---
        self.r = rtb.models.URDF.UR3()     # kinematics only; meshes not used
        self.q = self.r.qz.copy()          # current joint vector (rad)
        self.prev_qdot = np.zeros(6)       # for smoothing

        # --- ROS I/O ---
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.cam_sub = self.create_subscription(TwistStamped, '/ibvs/camera_twist', self.on_camera_twist, 10)
        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # Book-keeping
        self.have_js = False
        self.last_vc = np.zeros(6)
        self.last_twist_log_time = 0.0
        self.get_logger().info('IBVS->UR3 velocity node ready. Sub: /ibvs/camera_twist  Pub: %s' % self.cmd_topic)

        # Control timer (even if twist not streaming, we output zeros)
        self.timer = self.create_timer(1.0/self.rate_hz, self.control_step)

    # ----- Callbacks -----
    def on_joint_state(self, msg: JointState):
        # Map joint_states to our 6-DOF vector in the correct order.
        # UR3 typical order: shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3
        # Try to infer by name; if missing, fallback to first 6.
        names = list(msg.name)
        pos = np.array(msg.position, dtype=float)

        desired_order = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        q_new = np.zeros(6)
        ok = True
        for i, jname in enumerate(desired_order):
            if jname in names:
                q_new[i] = pos[names.index(jname)]
            else:
                ok = False
                break

        if not ok:
            # fallback: use first 6 positions
            if len(pos) >= 6:
                q_new = pos[:6]
            else:
                return  # not enough joints yet

        self.q = q_new
        self.have_js = True

    def on_camera_twist(self, msg: TwistStamped):
        # camera-frame twist ^cV
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_twist_log_time > 1.0:
            self.get_logger().info(f"[DEBUG] Received Twist: lin=({msg.twist.linear.x:.3f}, {msg.twist.linear.y:.3f}, {msg.twist.linear.z:.3f}) ang=({msg.twist.angular.x:.3f}, {msg.twist.angular.y:.3f}, {msg.twist.angular.z:.3f})")
            self.last_twist_log_time = now
        vc = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ], dtype=float)
        self.last_vc = vc

    # ----- Control step -----
    def control_step(self):
        if not self.have_js:
            # No joint states yet, publish zeros
            self.publish_qdot(np.zeros(6))
            return

        # 1) camera -> tool via hand–eye
        v_e = self.Ad_e_c @ self.last_vc

        # 2) tool -> base using current FK
        T0e = self.r.fkine(self.q)      # ^0T_e(q)
        Ad0e = adjoint_SE3(T0e)
        v_0 = Ad0e @ v_e                # ^0V

        # 3) Jacobian (base-frame) and DLS inverse
        J0 = self.r.jacob0(self.q)      # 6x6 for UR3
        JJt = J0 @ J0.T + (self.LAMBDA ** 2) * np.eye(6)
        qdot = J0.T @ np.linalg.solve(JJt, v_0)

        # 4) Smooth & clamp
        qdot = self.LPF_ALPHA * qdot + (1.0 - self.LPF_ALPHA) * self.prev_qdot
        qdot = np.clip(qdot, -self.QDOT_MAX, self.QDOT_MAX)
        self.prev_qdot = qdot

        # 5) Publish
        self.publish_qdot(qdot)

    def publish_qdot(self, qdot: np.ndarray):
        msg = Float64MultiArray()
        msg.data = qdot.tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = IBVStoUR3Vel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
