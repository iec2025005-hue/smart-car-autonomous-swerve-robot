#!/usr/bin/env python3
"""
audit_random_direction_suite.py -- Senior Engineer Random-Direction Test:
Tests 19 arbitrary motion angles across all 4 quadrants, logs displacement,
direction error, and yaw drift, and tests translation from multiple initial headings.
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class RandomDirectionAuditor(Node):
    def __init__(self):
        super().__init__('random_direction_auditor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.current_pose = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quat_to_yaw(ori)
        self.current_pose = (pos.x, pos.y, yaw)

    def stop_and_settle(self, duration: float = 0.5):
        t0 = time.time()
        while time.time() - t0 < duration:
            self.pub_cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

    def return_to_origin(self):
        # Closed-loop return to (0, 0)
        t_start = time.time()
        while time.time() - t_start < 4.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            if not self.current_pose:
                continue
            x, y, yaw = self.current_pose
            dist = math.hypot(x, y)
            if dist < 0.03:
                break
            angle = math.atan2(-y, -x)
            speed = min(0.35, max(0.12, dist * 0.8))
            cmd = Twist()
            # Command in robot frame (robot frame is approximately world frame for pure translation)
            # To be strictly frame-aware:
            # v_robot = R(-yaw) * v_world
            dx_w = -x
            dy_w = -y
            v_x = (dx_w * math.cos(yaw) + dy_w * math.sin(yaw))
            v_y = (-dx_w * math.sin(yaw) + dy_w * math.cos(yaw))
            norm = math.hypot(v_x, v_y)
            if norm > 1e-4:
                cmd.linear.x = speed * (v_x / norm)
                cmd.linear.y = speed * (v_y / norm)
            self.pub_cmd.publish(cmd)
            time.sleep(0.02)
        self.stop_and_settle(0.4)

    def rotate_to_heading(self, target_heading_deg: float):
        target_rad = math.radians(target_heading_deg)
        t0 = time.time()
        while time.time() - t0 < 4.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            if not self.current_pose:
                continue
            yaw = self.current_pose[2]
            err = normalize_angle(target_rad - yaw)
            if abs(err) < math.radians(1.5):
                break
            cmd = Twist()
            cmd.angular.z = max(-0.6, min(0.6, err * 1.5))
            self.pub_cmd.publish(cmd)
            time.sleep(0.02)
        self.stop_and_settle(0.4)

    def test_angle(self, angle_deg: float, speed: float = 0.30, duration: float = 1.6):
        self.stop_and_settle(0.3)
        rclpy.spin_once(self, timeout_sec=0.05)
        start_pose = self.current_pose

        ang_rad = math.radians(angle_deg)
        cmd = Twist()
        cmd.linear.x = speed * math.cos(ang_rad)
        cmd.linear.y = speed * math.sin(ang_rad)

        t0 = time.time()
        while time.time() - t0 < duration:
            self.pub_cmd.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        self.stop_and_settle(0.3)
        rclpy.spin_once(self, timeout_sec=0.05)
        end_pose = self.current_pose

        # Displacement in robot's initial frame:
        # Since this test commands /cmd_vel in chassis frame:
        # dx_robot, dy_robot
        dx = end_pose[0] - start_pose[0]
        dy = end_pose[1] - start_pose[1]
        dist = math.hypot(dx, dy)

        # In chassis initial frame:
        init_yaw = start_pose[2]
        dx_chassis = dx * math.cos(init_yaw) + dy * math.sin(init_yaw)
        dy_chassis = -dx * math.sin(init_yaw) + dy * math.cos(init_yaw)

        actual_dir_rad = math.atan2(dy_chassis, dx_chassis)
        actual_dir_deg = math.degrees(actual_dir_rad)
        expected_dir_deg = angle_deg

        dir_err_rad = normalize_angle(actual_dir_rad - ang_rad)
        dir_err_deg = abs(math.degrees(dir_err_rad))

        yaw_drift_rad = normalize_angle(end_pose[2] - start_pose[2])
        yaw_drift_deg = abs(math.degrees(yaw_drift_rad))

        passed = (dir_err_deg <= 3.0) and (yaw_drift_deg <= 2.5) and (dist > 0.25)

        # Reset to origin
        self.return_to_origin()

        return {
            'angle_deg': angle_deg,
            'dx': dx,
            'dy': dy,
            'dist': dist,
            'actual_dir_deg': actual_dir_deg,
            'dir_err_deg': dir_err_deg,
            'yaw_drift_deg': yaw_drift_deg,
            'passed': passed
        }

def run_random_direction_suite():
    rclpy.init()
    auditor = RandomDirectionAuditor()

    print("Waiting for /odom...")
    t_wait = time.time()
    while auditor.current_pose is None and time.time() - t_wait < 5.0:
        rclpy.spin_once(auditor, timeout_sec=0.05)

    print("=" * 80)
    print("  SENIOR ENGINEER REQUIREMENT: 19-ANGLE RANDOM DIRECTION AUDIT")
    print("=" * 80)

    test_angles = [
        0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0,
        120.0, 135.0, 150.0, 180.0, 210.0, 225.0, 240.0,
        270.0, 300.0, 315.0, 330.0, 345.0
    ]

    print(f"{'Angle':>7} | {'Displacement (dx, dy)':>23} | {'Dist':>6} | {'Actual Dir':>10} | {'Dir Err':>8} | {'Yaw Drift':>9} | {'Status':>6}")
    print("-" * 80)

    all_angles_passed = True
    results = []
    for deg in test_angles:
        res = auditor.test_angle(deg, speed=0.30, duration=1.5)
        results.append(res)
        if not res['passed']:
            all_angles_passed = False
        status_str = "✅ PASS" if res['passed'] else "❌ FAIL"
        disp_str = f"({res['dx']:+.3f}, {res['dy']:+.3f}) m"
        print(f"{res['angle_deg']:+6.1f}° | {disp_str:>23} | {res['dist']:5.3f}m | {res['actual_dir_deg']:+9.1f}° | {res['dir_err_deg']:7.2f}° | {res['yaw_drift_deg']:8.2f}° | {status_str}")

    print("-" * 80)
    max_dir_err = max(r['dir_err_deg'] for r in results)
    avg_dir_err = sum(r['dir_err_deg'] for r in results) / len(results)
    max_yaw_drift = max(r['yaw_drift_deg'] for r in results)
    avg_yaw_drift = sum(r['yaw_drift_deg'] for r in results) / len(results)
    print(f"Max Direction Error: {max_dir_err:.2f}° | Average Direction Error: {avg_dir_err:.2f}°")
    print(f"Max Yaw Drift      : {max_yaw_drift:.2f}° | Average Yaw Drift      : {avg_yaw_drift:.2f}°")

    print("\n" + "=" * 80)
    print("  TEST TRANSLATION FROM NON-ZERO INITIAL CHASSIS HEADINGS")
    print("=" * 80)

    heading_tests = [
        (+45.0, 90.0, "Chassis at +45°, command lateral strafe 90°"),
        (+90.0, 0.0, "Chassis at +90°, command forward translation 0°"),
        (-60.0, 135.0, "Chassis at -60°, command diagonal 135°"),
    ]

    all_headings_passed = True
    for init_hdg, trans_dir, desc in heading_tests:
        print(f"\n[Setting Chassis Heading: {init_hdg:+5.1f}° -> {desc}]")
        auditor.rotate_to_heading(init_hdg)
        res = auditor.test_angle(trans_dir, speed=0.30, duration=1.5)
        passed = (res['dir_err_deg'] <= 3.0) and (res['yaw_drift_deg'] <= 2.5)
        if not passed:
            all_headings_passed = False
        status_str = "✅ PASS" if passed else "❌ FAIL"
        disp_str = f"({res['dx']:+.3f}, {res['dy']:+.3f}) m"
        print(f"  Commanded Dir in Chassis Frame: {trans_dir:+5.1f}°")
        print(f"  Resulting Displacement        : {disp_str} (dist={res['dist']:.3f}m)")
        print(f"  Actual Motion Direction       : {res['actual_dir_deg']:+5.1f}° (Error = {res['dir_err_deg']:.2f}°)")
        print(f"  Chassis Yaw Drift             : {res['yaw_drift_deg']:.2f}° [{status_str}]")

    # Final return to origin and 0 heading
    auditor.rotate_to_heading(0.0)
    auditor.return_to_origin()

    auditor.destroy_node()
    rclpy.shutdown()

    overall = all_angles_passed and all_headings_passed
    print("\n" + "=" * 80)
    print(f"  OVERALL RANDOM-DIRECTION SUITE STATUS: {'✅ ALL TESTS PASSED' if overall else '❌ SOME TESTS FAILED'}")
    print("=" * 80)

if __name__ == '__main__':
    run_random_direction_suite()
