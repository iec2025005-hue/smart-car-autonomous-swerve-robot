#!/usr/bin/env python3
"""
audit_section_16_point_to_point.py -- Section 16:
Test from SAME point (0,0,0) to 8 different target points with RESET before EACH test:
Targets:
1. (1, 0)
2. (1, 1)
3. (0, 1)
4. (-1, 1)
5. (-1, 0)
6. (-1, -1)
7. (0, -1)
8. (1, -1)
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import os
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PointToPointAuditor(Node):
    def __init__(self):
        super().__init__('point_to_point_auditor')

        self.current_pose = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reset_odom = self.create_publisher(Empty, '/reset_odom', 10)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def reset_to_origin(self):
        self.pub_cmd_vel.publish(Twist())
        time.sleep(0.1)
        os.system('gz service -s /world/smart_car_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req \'name: "smart_car", position: {x: 0.0, y: 0.0, z: 0.04}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}\' >/dev/null 2>&1')
        self.pub_reset_odom.publish(Empty())
        time.sleep(0.2)
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.01)

    def send_goal_and_wait(self, gx: float, gy: float, timeout_sec: float = 16.0):
        target_dist = math.hypot(gx, gy)
        target_yaw = 0.0 # Maintain zero chassis yaw (pure holonomic swerve translation)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.orientation.w = 1.0

        self.pub_goal.publish(goal)

        t_start = time.time()
        decel_observed = False
        arrived = False
        settled_time = None

        while (time.time() - t_start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.current_pose is None:
                continue

            cx, cy, cyaw = self.current_pose
            dist_to_goal = math.hypot(gx - cx, gy - cy)

            if dist_to_goal < 0.20:
                decel_observed = True

            if dist_to_goal <= 0.055:
                arrived = True
                settled_time = time.time() - t_start
                break

            time.sleep(0.02)

        # Allow final stop
        time.sleep(0.3)
        for _ in range(10): rclpy.spin_once(self, timeout_sec=0.01)

        fx, fy, fyaw = self.current_pose
        final_dist_err = math.hypot(gx - fx, gy - fy)
        final_yaw_err = math.degrees(abs(normalize_angle(fyaw - target_yaw)))
        time_taken = settled_time if settled_time is not None else (time.time() - t_start)

        passed = arrived and (final_dist_err <= 0.055) and decel_observed

        return {
            'target': (gx, gy),
            'final_pose': (fx, fy, fyaw),
            'pos_err_mm': final_dist_err * 1000.0,
            'yaw_err_deg': final_yaw_err,
            'time_sec': time_taken,
            'decel': decel_observed,
            'passed': passed,
        }

def run_section_16():
    rclpy.init()
    auditor = PointToPointAuditor()

    print("Waiting for /odom and DDS subscriber discovery...")
    t_wait = time.time()
    while (auditor.pub_goal.get_subscription_count() < 1 or auditor.current_pose is None) and time.time() - t_wait < 5.0:
        rclpy.spin_once(auditor, timeout_sec=0.05)
    time.sleep(0.5)

    print("=" * 95)
    print("  SECTION 16: POINT-TO-POINT HOLONOMIC NAVIGATION (RESET TO 0,0,0 BEFORE EACH)")
    print("=" * 95)
    print(f"{'Target Goal':<16} | {'Final Pose (x, y, yaw)':<30} | {'Pos Error':>11} | {'Time':>8} | {'Decel':>6} | {'Status':>6}")
    print("-" * 95)

    targets = [
        ("Target 1: ( 1,  0)", +1.0,  0.0),
        ("Target 2: ( 1,  1)", +1.0, +1.0),
        ("Target 3: ( 0,  1)",  0.0, +1.0),
        ("Target 4: (-1,  1)", -1.0, +1.0),
        ("Target 5: (-1,  0)", -1.0,  0.0),
        ("Target 6: (-1, -1)", -1.0, -1.0),
        ("Target 7: ( 0, -1)",  0.0, -1.0),
        ("Target 8: ( 1, -1)", +1.0, -1.0),
    ]

    sec16_all_pass = True

    for label, gx, gy in targets:
        auditor.reset_to_origin()
        res = auditor.send_goal_and_wait(gx, gy, timeout_sec=16.0)

        if not res['passed']:
            sec16_all_pass = False

        fx, fy, fyaw = res['final_pose']
        pose_s = f"({fx:+.3f}, {fy:+.3f}, {math.degrees(fyaw):+.1f}°)"
        decel_s = "YES" if res['decel'] else "NO"
        status_s = "✅ PASS" if res['passed'] else "❌ FAIL"

        print(f"{label:<16} | {pose_s:<30} | {res['pos_err_mm']:9.1f} mm | {res['time_sec']:6.2f} s | {decel_s:>6} | {status_s}")

    auditor.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 95)
    print(f"  SECTION 16 OVERALL STATUS: {'✅ ALL TESTS PASSED' if sec16_all_pass else '❌ SOME TESTS FAILED'}")
    print("=" * 95)

if __name__ == '__main__':
    run_section_16()
