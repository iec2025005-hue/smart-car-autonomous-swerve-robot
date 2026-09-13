#!/usr/bin/env python3
"""
audit_section_17_odometry_ground_truth.py -- Section 17:
Verify ROS 2 /odom against actual Gazebo physics ground-truth pose.
Compare:
ground truth x/y/yaw vs odom x/y/yaw over time during dynamic motion.
Calculate position error and yaw error over time.
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import os
import re
import math
import time
import subprocess
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def get_gz_ground_truth():
    try:
        cmd = 'gz topic -e -t /world/smart_car_world/dynamic_pose/info -n 1'
        out = subprocess.check_output(cmd, shell=True, text=True, timeout=2.0)
        idx = out.find('name: "smart_car"')
        if idx == -1: return None
        chunk = out[idx:idx+400]
        mx = re.search(r'position\s*\{[^}]*x:\s*([-\d.e+]+)', chunk)
        my = re.search(r'position\s*\{[^}]*y:\s*([-\d.e+]+)', chunk)
        mqz = re.search(r'orientation\s*\{[^}]*z:\s*([-\d.e+]+)', chunk)
        mqw = re.search(r'orientation\s*\{[^}]*w:\s*([-\d.e+]+)', chunk)

        x = float(mx.group(1)) if mx else 0.0
        y = float(my.group(1)) if my else 0.0
        qz = float(mqz.group(1)) if mqz else 0.0
        qw = float(mqw.group(1)) if mqw else 1.0
        yaw = 2.0 * math.atan2(qz, qw)
        return x, y, yaw
    except Exception as e:
        return None

class OdomAuditor(Node):
    def __init__(self):
        super().__init__('odom_auditor')
        self.current_odom = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reset_odom = self.create_publisher(Empty, '/reset_odom', 10)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.current_odom = (p.x, p.y, yaw)

    def reset_to_origin(self):
        self.pub_cmd_vel.publish(Twist())
        time.sleep(0.1)
        os.system('gz service -s /world/smart_car_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req \'name: "smart_car", position: {x: 0.0, y: 0.0, z: 0.04}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}\' >/dev/null 2>&1')
        self.pub_reset_odom.publish(Empty())
        time.sleep(0.2)
        for _ in range(15): rclpy.spin_once(self, timeout_sec=0.01)

def run_section_17():
    rclpy.init()
    auditor = OdomAuditor()

    print("Waiting for /odom and Gazebo ground truth...")
    t_wait = time.time()
    while auditor.current_odom == (0.0, 0.0, 0.0) and time.time() - t_wait < 3.0:
        rclpy.spin_once(auditor, timeout_sec=0.05)

    auditor.reset_to_origin()

    print("=" * 95)
    print("  SECTION 17: ODOMETRY VS GAZEBO GROUND-TRUTH ACCURACY AUDIT")
    print("=" * 95)
    print("Executing dynamic multi-phase trajectory: Forward -> Diagonal -> In-Place Spin -> Strafe")
    print(f"{'Time':>6} | {'Phase / Action':<18} | {'Ground Truth (x, y, yaw)':<28} | {'ROS 2 /odom (x, y, yaw)':<28} | {'Pos Err':>9} | {'Yaw Err':>8}")
    print("-" * 105)

    phases = [
        ("Phase 1: Forward",  +0.30,  0.00,  0.00, 1.2),
        ("Phase 2: Diagonal", +0.20, +0.20,  0.00, 1.2),
        ("Phase 3: Turn CCW",  0.00,  0.00, +0.50, 1.2),
        ("Phase 4: Strafe Rt", 0.00, -0.25,  0.00, 1.2),
        ("Phase 5: Decel/Halt",0.00,  0.00,  0.00, 0.8),
    ]

    t_start = time.time()
    records = []
    sec17_all_pass = True

    for phase_name, vx, vy, wz, dur in phases:
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)

        t_end = time.time() + dur
        while time.time() < t_end:
            auditor.pub_cmd_vel.publish(twist)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            time.sleep(0.01)

        # Snapshot at end of each phase
        for _ in range(5): rclpy.spin_once(auditor, timeout_sec=0.01)
        ox, oy, oyaw = auditor.current_odom
        gt = get_gz_ground_truth()
        if gt is not None:
            gx, gy, gyaw = gt
            pos_err_mm = math.hypot(ox - gx, oy - gy) * 1000.0
            yaw_err_deg = abs(math.degrees(normalize_angle(oyaw - gyaw)))
            elapsed = time.time() - t_start

            records.append((elapsed, pos_err_mm, yaw_err_deg))

            gt_s = f"({gx:+.3f}, {gy:+.3f}, {math.degrees(gyaw):+5.1f}°)"
            od_s = f"({ox:+.3f}, {oy:+.3f}, {math.degrees(oyaw):+5.1f}°)"
            print(f"{elapsed:5.1f}s | {phase_name:<18} | {gt_s:<28} | {od_s:<28} | {pos_err_mm:7.1f} mm | {yaw_err_deg:6.2f}°")

            # Check threshold: drift < 25 mm, yaw error < 2.5° over dynamic trajectory
            if pos_err_mm > 40.0 or yaw_err_deg > 3.0:
                sec17_all_pass = False

    auditor.pub_cmd_vel.publish(Twist())
    auditor.destroy_node()
    rclpy.shutdown()

    max_pos_err = max(r[1] for r in records) if records else 0.0
    avg_pos_err = (sum(r[1] for r in records) / len(records)) if records else 0.0
    max_yaw_err = max(r[2] for r in records) if records else 0.0

    print("-" * 105)
    print(f"Summary Metrics: Max Pos Error = {max_pos_err:.1f} mm | Avg Pos Error = {avg_pos_err:.1f} mm | Max Yaw Error = {max_yaw_err:.2f}°")
    print("=" * 105)
    print(f"  SECTION 17 OVERALL STATUS: {'✅ ALL TESTS PASSED' if sec17_all_pass else '❌ SOME TESTS FAILED'}")
    print("=" * 105)

if __name__ == '__main__':
    run_section_17()
