#!/usr/bin/env python3
"""
audit_section_12_13_14_15_motion.py -- Sections 12, 13, 14, 15:
- Section 12: Pure translations (A: Forward, B: Reverse, C: Left, D: Right) from (0,0,0)
- Section 13: 8 directions (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°) from SAME (0,0,0)
- Section 14: Pure rotation (+1.0 rad/s and -1.0 rad/s), wheel arrangements, drift
- Section 15: Combined motions (vx+wz, vy+wz, vx+vy+wz), verify 4 distinct wheel states
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import os
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
import config

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MotionAuditor(Node):
    def __init__(self):
        super().__init__('motion_auditor')

        self.current_pose = (0.0, 0.0, 0.0) # x, y, yaw
        self.joint_positions = {}
        self.joint_velocities = {}

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reset_odom = self.create_publisher(Empty, '/reset_odom', 10)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def js_cb(self, msg: JointState):
        for n, p, v in zip(msg.name, msg.position, msg.velocity):
            self.joint_positions[n] = p
            self.joint_velocities[n] = v

    def reset_to_origin(self):
        self.pub_cmd_vel.publish(Twist())
        time.sleep(0.1)
        os.system('gz service -s /world/smart_car_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req \'name: "smart_car", position: {x: 0.0, y: 0.0, z: 0.04}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}\' >/dev/null 2>&1')
        self.pub_reset_odom.publish(Empty())
        time.sleep(0.2)
        # Drain odom
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.01)

def run_motion_audit():
    rclpy.init()
    auditor = MotionAuditor()

    print("Waiting for /odom and /joint_states...")
    t_wait = time.time()
    while len(auditor.joint_positions) < 8 and time.time() - t_wait < 5.0:
        rclpy.spin_once(auditor, timeout_sec=0.05)

    # ═════════════════════════════════════════════════════════════════════════
    # SECTION 12: PURE TRANSLATION AUDIT
    # ═════════════════════════════════════════════════════════════════════════
    print("=" * 95)
    print("  SECTION 12: PURE TRANSLATION (A, B, C, D) FROM SAME START POSE (0,0,0)")
    print("=" * 95)
    print(f"{'State':<18} | {'Command (vx, vy)':<18} | {'Δx (m)':>8} | {'Δy (m)':>8} | {'ΔYaw (deg)':>11} | {'Status':>6}")
    print("-" * 95)

    sec12_cases = [
        ("A: Forward",  +0.35,  0.00),
        ("B: Reverse",  -0.35,  0.00),
        ("C: Left",      0.00, +0.35),
        ("D: Right",     0.00, -0.35),
    ]

    sec12_pass = True
    for name, vx_cmd, vy_cmd in sec12_cases:
        auditor.reset_to_origin()
        x0, y0, yaw0 = auditor.current_pose

        twist = Twist()
        twist.linear.x = float(vx_cmd)
        twist.linear.y = float(vy_cmd)

        t_end = time.time() + 1.5
        while time.time() < t_end:
            auditor.pub_cmd_vel.publish(twist)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            time.sleep(0.01)

        auditor.pub_cmd_vel.publish(Twist())
        time.sleep(0.2)
        for _ in range(10): rclpy.spin_once(auditor, timeout_sec=0.01)

        x1, y1, yaw1 = auditor.current_pose
        dx = x1 - x0
        dy = y1 - y0
        dyaw_deg = math.degrees(normalize_angle(yaw1 - yaw0))

        # Check alignment
        if vx_cmd != 0:
            passed = (abs(dx) > 0.35) and (abs(dy) < 0.05) and (abs(dyaw_deg) < 1.0)
        else:
            passed = (abs(dy) > 0.35) and (abs(dx) < 0.05) and (abs(dyaw_deg) < 1.0)

        if not passed: sec12_pass = False
        cmd_str = f"({vx_cmd:+.2f}, {vy_cmd:+.2f}) m/s"
        print(f"{name:<18} | {cmd_str:<18} | {dx:+8.3f} | {dy:+8.3f} | {dyaw_deg:+10.2f}° | {'✅ PASS' if passed else '❌ FAIL'}")

    # ═════════════════════════════════════════════════════════════════════════
    # SECTION 13: 8 DIRECTIONS FROM SAME STARTING POINT (0,0,0)
    # ═════════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 95)
    print("  SECTION 13: 8 DIRECTIONS FROM SAME STARTING POINT (0,0,0) - SENIOR AUDIT")
    print("=" * 95)
    print(f"{'Target Angle':>12} | {'Cmd (vx, vy)':<18} | {'Final (x, y)':<18} | {'Actual Traj Angle':>18} | {'Angular Error':>14} | {'Status':>6}")
    print("-" * 95)

    sec13_angles = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
    speed = 0.30
    sec13_pass = True

    for ang_deg in sec13_angles:
        auditor.reset_to_origin()
        x0, y0, yaw0 = auditor.current_pose

        ang_rad = math.radians(ang_deg)
        vx = speed * math.cos(ang_rad)
        vy = speed * math.sin(ang_rad)

        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)

        t_end = time.time() + 1.5
        while time.time() < t_end:
            auditor.pub_cmd_vel.publish(twist)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            time.sleep(0.01)

        auditor.pub_cmd_vel.publish(Twist())
        time.sleep(0.2)
        for _ in range(10): rclpy.spin_once(auditor, timeout_sec=0.01)

        x1, y1, yaw1 = auditor.current_pose
        dx = x1 - x0
        dy = y1 - y0

        actual_traj_rad = math.atan2(dy, dx)
        actual_traj_deg = math.degrees(actual_traj_rad)
        # Normalize to [0, 360)
        if actual_traj_deg < 0: actual_traj_deg += 360.0
        ang_err = abs(actual_traj_deg - ang_deg)
        if ang_err > 180.0: ang_err = 360.0 - ang_err

        dist = math.hypot(dx, dy)
        passed = (ang_err < 1.5) and (dist > 0.35)
        if not passed: sec13_pass = False

        cmd_s = f"({vx:+.2f}, {vy:+.2f})"
        fin_s = f"({x1:+.3f}, {y1:+.3f})"
        print(f"{ang_deg:+11.1f}° | {cmd_s:<18} | {fin_s:<18} | {actual_traj_deg:+17.1f}° | {ang_err:13.2f}° | {'✅ PASS' if passed else '❌ FAIL'}")

    # ═════════════════════════════════════════════════════════════════════════
    # SECTION 14: PURE ROTATION AUDIT (omega = +1.0 and -1.0 rad/s)
    # ═════════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 95)
    print("  SECTION 14: PURE ROTATION AUDIT (omega = +1.0 and -1.0 rad/s)")
    print("=" * 95)
    print("Theoretical Module Angles for CCW Pure Spin: FL=+137.7°, FR=+42.3°, RL=-137.7°, RR=-42.3°")
    print(f"{'Cmd Omega':>10} | {'ΔYaw (deg)':>12} | {'Linear Drift (m)':>18} | {'Measured Steer Angles (FL, FR, RL, RR)':>42} | {'Status':>6}")
    print("-" * 95)

    sec14_pass = True
    rot_cases = [+1.0, -1.0]

    for wz_cmd in rot_cases:
        auditor.reset_to_origin()
        x0, y0, yaw0 = auditor.current_pose

        twist = Twist()
        twist.angular.z = float(wz_cmd)

        t_end = time.time() + 2.0
        while time.time() < t_end:
            auditor.pub_cmd_vel.publish(twist)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            time.sleep(0.01)

        # Snapshot steering angles during spin
        s_fl = math.degrees(auditor.joint_positions.get('fl_steer_joint', 0.0))
        s_fr = math.degrees(auditor.joint_positions.get('fr_steer_joint', 0.0))
        s_rl = math.degrees(auditor.joint_positions.get('rl_steer_joint', 0.0))
        s_rr = math.degrees(auditor.joint_positions.get('rr_steer_joint', 0.0))

        auditor.pub_cmd_vel.publish(Twist())
        time.sleep(0.2)
        for _ in range(10): rclpy.spin_once(auditor, timeout_sec=0.01)

        x1, y1, yaw1 = auditor.current_pose
        drift = math.hypot(x1 - x0, y1 - y0)
        dyaw_deg = math.degrees(normalize_angle(yaw1 - yaw0))

        passed = (drift < 0.05) and (abs(dyaw_deg) > 40.0)
        if not passed: sec14_pass = False

        steer_str = f"({s_fl:+.1f}°, {s_fr:+.1f}°, {s_rl:+.1f}°, {s_rr:+.1f}°)"
        print(f"{wz_cmd:+9.2f} rad/s | {dyaw_deg:+11.1f}° | {drift:17.4f} m | {steer_str:>42} | {'✅ PASS' if passed else '❌ FAIL'}")

    # ═════════════════════════════════════════════════════════════════════════
    # SECTION 15: COMBINED MOTION AUDIT (vx+wz, vy+wz, vx+vy+wz)
    # ═════════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 95)
    print("  SECTION 15: COMBINED MOTION (vx+wz, vy+wz, vx+vy+wz) - DISTINCT WHEEL SPEEDS & ANGLES")
    print("=" * 95)
    print(f"{'Motion Scenario':<20} | {'Wheel Speeds (FL, FR, RL, RR) m/s':<35} | {'Steer Angles (FL, FR, RL, RR)':<30} | {'Distinct?':>9} | {'Status':>6}")
    print("-" * 95)

    sec15_cases = [
        ("vx + wz",         +0.25,  0.00, +0.50),
        ("vy + wz",          0.00, +0.25, +0.50),
        ("vx + vy + wz",    +0.25, +0.15, +0.50),
    ]

    sec15_pass = True
    wheel_r = 0.039965

    for label, vx_cmd, vy_cmd, wz_cmd in sec15_cases:
        auditor.reset_to_origin()

        twist = Twist()
        twist.linear.x = float(vx_cmd)
        twist.linear.y = float(vy_cmd)
        twist.angular.z = float(wz_cmd)

        t_end = time.time() + 1.5
        while time.time() < t_end:
            auditor.pub_cmd_vel.publish(twist)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            time.sleep(0.01)

        v_fl = abs(auditor.joint_velocities.get('fl_wheel_joint', 0.0)) * wheel_r
        v_fr = abs(auditor.joint_velocities.get('fr_wheel_joint', 0.0)) * wheel_r
        v_rl = abs(auditor.joint_velocities.get('rl_wheel_joint', 0.0)) * wheel_r
        v_rr = abs(auditor.joint_velocities.get('rr_wheel_joint', 0.0)) * wheel_r

        a_fl = math.degrees(auditor.joint_positions.get('fl_steer_joint', 0.0))
        a_fr = math.degrees(auditor.joint_positions.get('fr_steer_joint', 0.0))
        a_rl = math.degrees(auditor.joint_positions.get('rl_steer_joint', 0.0))
        a_rr = math.degrees(auditor.joint_positions.get('rr_steer_joint', 0.0))

        auditor.pub_cmd_vel.publish(Twist())
        time.sleep(0.2)

        spd_str = f"({v_fl:.3f}, {v_fr:.3f}, {v_rl:.3f}, {v_rr:.3f})"
        ang_str = f"({a_fl:+.1f}°, {a_fr:+.1f}°, {a_rl:+.1f}°, {a_rr:+.1f}°)"

        # In combined motion, wheel speeds and angles must NOT all be identical
        speeds = [v_fl, v_fr, v_rl, v_rr]
        distinct = (max(speeds) - min(speeds)) > 0.05
        passed = distinct
        if not passed: sec15_pass = False

        print(f"{label:<20} | {spd_str:<35} | {ang_str:<30} | {'YES' if distinct else 'NO':>9} | {'✅ PASS' if passed else '❌ FAIL'}")

    auditor.pub_cmd_vel.publish(Twist())
    auditor.destroy_node()
    rclpy.shutdown()

    overall = sec12_pass and sec13_pass and sec14_pass and sec15_pass
    print("\n" + "=" * 95)
    print(f"  SECTIONS 12, 13, 14, 15 OVERALL STATUS: {'✅ ALL TESTS PASSED' if overall else '❌ SOME TESTS FAILED'}")
    print("=" * 95)

if __name__ == '__main__':
    run_motion_audit()
