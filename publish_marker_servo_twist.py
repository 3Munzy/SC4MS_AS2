#!/usr/bin/env python3
"""
publish_marker_servo_twist.py
PBVS toward a Marker, with tool-frame angular control on TOOL-Y to recruit wrist_2.

- Subscribes: /visualization_marker (visualization_msgs/Marker)
- Subscribes: /joint_states (sensor_msgs/JointState)
- Publishes:  /ibvs/camera_twist (geometry_msgs/TwistStamped)

Behavior:
  * Uses UR3 FK (roboticstoolbox-python) from joint_states to get EE pose in `base`.
  * Assumes marker pose is expressed in `base`.
  * Drives the EE to a standoff `target_distance` along marker +Z.
  * Orientation target = marker orientation * R_offset.
  * Publishes a BODY twist in `tool0` (default) with ANGULAR CONTROL ONLY ON TOOL-Y.
    This excites wrist_2_joint on UR3/UR3e.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb


class MarkerServoTwistPublisher(Node):
    def __init__(self):
        super().__init__('marker_servo_twist_publisher')

        # -------- Config --------
        self.base_frame = 'base'
        self.ee_frame   = 'tool0'

        self.target_distance   = 0.50     # standoff along marker +Z (meters)
        self.align_with_marker = True     # align tool orientation to marker * R_offset
        self.publish_body_twist = True    # True -> publish in tool frame ('tool0')

        # Keep only TOOL-Y angular component (strongly maps to wrist_2 on UR3/UR3e)
        self.keep_tool_y_only = True

        # Small constant bias about TOOL-Y (deg). 3–6° gently wakes wrist_2; 0 disables
        self.wrist2_bias_deg = 4.0

        # Orientation offset (marker frame -> desired tool frame).
        # Calibrate once at a "looks right" pose: R_offset = Rm.T @ Re
        self.R_offset = np.eye(3)

        # Gains / limits
        self.Kp = 1.0
        self.Ko = 1.2
        self.v_max = 0.05        # m/s
        self.w_max = 0.50        # rad/s
        self.deadband_pos = 0.005   # m
        self.deadband_ang = 0.01    # rad (applies to tool-Y component)

        # -------- IO --------
        self.marker_sub = self.create_subscription(Marker, '/visualization_marker', self.on_marker, 10)
        self.js_sub     = self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.twist_pub  = self.create_publisher(TwistStamped, '/ibvs/camera_twist', 10)

        # -------- Robot model & state (for FK) --------
        self.ur3 = rtb.models.URDF.UR3()   # uses internal DH/URDF model
        self.current_q = self.ur3.qz.copy()
        self.have_js = False

        # housekeeping
        self.stopped = False
        self.last_twist = None
        self.timer = self.create_timer(0.05, self.publish_zero_twist)

        self.get_logger().info('PBVS: tool-Y angular control (body twist) to excite wrist_2.')

    # ---------- helpers ----------
    @staticmethod
    def quat_to_R(x, y, z, w):
        q = np.array([x, y, z, w], dtype=float)
        n = np.linalg.norm(q)
        if n < 1e-12:
            return np.eye(3)
        x, y, z, w = q / n
        w2, x2, y2, z2 = w*w, x*x, y*y, z*z
        return np.array([
            [1-2*(y2+z2),   2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1-2*(x2+z2),   2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x2+y2)]
        ], dtype=float)

    @staticmethod
    def log_so3(R):
        c = np.clip((np.trace(R)-1)/2.0, -1.0, 1.0)
        th = np.arccos(c)
        if th < 1e-8:
            return np.zeros(3)
        w = (1/(2*np.sin(th))) * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
        return th * w

    @staticmethod
    def skew(v):
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]], dtype=float)

    # ---------- callbacks ----------
    def on_joint_state(self, msg: JointState):
        names = list(msg.name)
        pos = np.array(msg.position, dtype=float)
        order = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        q = np.zeros(6)
        ok = True
        for i, j in enumerate(order):
            if j in names:
                q[i] = pos[names.index(j)]
            else:
                ok = False; break
        if not ok:
            if len(pos) >= 6:
                q = pos[:6]
            else:
                return
        self.current_q = q
        self.have_js = True

    def on_marker(self, msg: Marker):
        if not self.have_js:
            self.get_logger().warn('No joint_states yet; PBVS skipped.')
            return

        # 1) Marker pose in base
        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        Rm = self.quat_to_R(qx, qy, qz, qw)
        pm = np.array([px, py, pz])

        # 2) Current EE pose from FK
        T0e = self.ur3.fkine(self.current_q)
        pe = T0e.t.flatten()      # EE position in base
        Re = T0e.R                # EE rotation in base

        # 3) Desired pose
        if self.align_with_marker:
            R_des = Rm @ self.R_offset
            p_des = pm + Rm @ np.array([0.0, 0.0, self.target_distance])  # along marker +Z
        else:
            R_des = Re
            p_des = pm + Rm @ np.array([0.0, 0.0, self.target_distance])

        # 4) Pose error
        ep = p_des - pe                          # position error in base
        R_err = R_des @ Re.T
        e_w_base = self.log_so3(R_err)           # angular error in base

        # Express angular error in TOOL frame and (optionally) keep only TOOL-Y
        e_w_body = Re.T @ e_w_base               # base -> tool
        if self.keep_tool_y_only:
            e_w_body = np.array([0.0, e_w_body[1], 0.0])

        # Add gentle TOOL-Y bias to wake wrist_2
        if abs(self.wrist2_bias_deg) > 1e-6:
            e_w_body[1] += np.deg2rad(self.wrist2_bias_deg) * 0.25

        # 5) Deadbands (pos in base, ang only tool-Y)
        if np.linalg.norm(ep) < self.deadband_pos and abs(e_w_body[1]) < self.deadband_ang:
            self.publish_twist(np.zeros(3), np.zeros(3))
            self.stopped = True
            return

        # 6) P control (spatial linear, body angular-Y)
        v_base = self.Kp * ep                     # linear twist (base)
        w_body = self.Ko * e_w_body               # angular twist (tool), only Y active

        # Convert for publishing
        if self.publish_body_twist:
            # Need body linear: v_b = R^T (v - p×w), with w in base:
            w_base = Re @ w_body
            v_body = Re.T @ (v_base - np.cross(pe, w_base))
            v_out, w_out = v_body, w_body
            frame_id = self.ee_frame
        else:
            # Publish spatial twist in base (keep only the mapped component of w)
            w_out = Re @ w_body
            v_out = v_base
            frame_id = self.base_frame

        # Saturate
        nv, nw = np.linalg.norm(v_out), np.linalg.norm(w_out)
        if nv > self.v_max:
            v_out *= self.v_max / (nv + 1e-12)
        if nw > self.w_max:
            w_out *= self.w_max / (nw + 1e-12)

        # 7) Publish
        self.publish_twist(v_out, w_out, frame=frame_id)
        self.last_twist = (v_out, w_out)
        self.stopped = False

        self.get_logger().info(
            f"PBVS: |ep|={np.linalg.norm(ep):.3f} m, |ew_tool|={np.linalg.norm(e_w_body):.3f} rad, "
            f"v=({v_out[0]:.3f},{v_out[1]:.3f},{v_out[2]:.3f}) w=({w_out[0]:.3f},{w_out[1]:.3f},{w_out[2]:.3f})"
        )

    # ---------- publishing ----------
    def publish_twist(self, v, w, frame=None):
        if frame is None:
            frame = self.ee_frame if self.publish_body_twist else self.base_frame
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z  = float(v[0]), float(v[1]), float(v[2])
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = float(w[0]), float(w[1]), float(w[2])
        self.twist_pub.publish(msg)

    def publish_zero_twist(self):
        if self.stopped or self.last_twist is None:
            frame = self.ee_frame if self.publish_body_twist else self.base_frame
            self.publish_twist(np.zeros(3), np.zeros(3), frame=frame)


def main():
    rclpy.init()
    node = MarkerServoTwistPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
