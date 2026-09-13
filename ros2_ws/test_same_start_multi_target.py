#!/usr/bin/env python3
"""
test_same_start_multi_target.py -- Verify robot starting from the SAME position
and traveling toward different target positions without accumulating unexpected heading/drift errors.
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
from std_msgs.msg import Empty

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MultiTargetValidator(Node):
    def __init__(self):
        super().__init__('multi_target_validator')
        self.current_pose = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
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
        for _ in range(15): rclpy.spin_once(self, timeout_sec=0.01)

    def drive_to_target(self, target_x, target_y, speed=0.35, timeout_sec=6.0):
        self.reset_to_origin()
        x0, y0, yaw0 = self.current_pose
        
        target_dist = math.hypot(target_x, target_y)
        target_bearing = math.atan2(target_y, target_x)

        t_start = time.time()
        drift_max = 0.0

        while time.time() - t_start < timeout_sec:
            x, y, yaw = self.current_pose
            dx = target_x - x
            dy = target_y - y
            dist_rem = math.hypot(dx, dy)

            # Lateral cross-track error from line (0,0)->(target_x, target_y)
            crosstrack = abs(x * target_y - y * target_x) / target_dist
            if crosstrack > drift_max:
                drift_max = crosstrack

            if dist_rem < 0.04:
                break

            cmd_speed = min(speed, max(0.08, dist_rem * 1.0))
            theta = math.atan2(dy, dx)
            # Relative command in robot body frame
            body_vx = cmd_speed * math.cos(theta - yaw)
            body_vy = cmd_speed * math.sin(theta - yaw)

            cmd = Twist()
            cmd.linear.x = body_vx
            cmd.linear.y = body_vy
            cmd.angular.z = 0.0 # Maintain zero heading
            self.pub_cmd_vel.publish(cmd)

            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        self.pub_cmd_vel.publish(Twist())
        time.sleep(0.2)
        for _ in range(10): rclpy.spin_once(self, timeout_sec=0.01)

        xf, yf, yawf = self.current_pose
        final_err = math.hypot(target_x - xf, target_y - yf)
        heading_err_deg = math.degrees(abs(normalize_angle(yawf - yaw0)))

        return final_err, drift_max, heading_err_deg

def main():
    rclpy.init()
    node = MultiTargetValidator()
    print("Waiting for odom...")
    time.sleep(1.0)
    for _ in range(10): rclpy.spin_once(node, timeout_sec=0.05)

    targets = [
        ("Target 1 (East)",      +0.80,  0.00),
        ("Target 2 (North)",      0.00, +0.80),
        ("Target 3 (North-East)",+0.60, +0.60),
        ("Target 4 (North-West)",-0.60, +0.60),
        ("Target 5 (South)",      0.00, -0.80),
        ("Target 6 (South-East)",+0.60, -0.60),
        ("Target 7 (West)",      -0.80,  0.00),
        ("Target 8 (South-West)",-0.60, -0.60),
    ]

    print("\n" + "=" * 105)
    print("  SAME-START MULTI-TARGET REPEATABILITY BENCHMARK (All runs start from (0,0,0))")
    print("=" * 105)
    print(f"{'Target Name':<24} | {'Target Pos (m)':<18} | {'Arrival Err':>12} | {'Max Drift':>11} | {'Heading Drift':>14} | {'Status':>6}")
    print("-" * 105)

    all_passed = True
    for name, tx, ty in targets:
        arr_err, drift, h_err = node.drive_to_target(tx, ty)
        passed = (arr_err < 0.05) and (drift < 0.035) and (h_err < 1.0)
        if not passed: all_passed = False
        t_str = f"({tx:+.2f}, {ty:+.2f})"
        print(f"{name:<24} | {t_str:<18} | {arr_err*1000:9.1f} mm | {drift*1000:8.1f} mm | {h_err:11.2f}° | {'✅ PASS' if passed else '❌ FAIL'}")

    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 105)
    print(f"  REPEATABILITY BENCHMARK RESULT: {'✅ ALL TARGETS PASSED WITH EXCELLENT REPEATABILITY' if all_passed else '❌ BENCHMARK FAILED'}")
    print("=" * 105)

if __name__ == '__main__':
    main()
