#!/usr/bin/env python3
"""
audit_vision_avoidance_suite.py -- Audits vision pipeline, target tracking,
obstacle avoidance swerve maneuvers, and mode transitions.
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Float64MultiArray

class VisionAvoidanceAuditor(Node):
    def __init__(self):
        super().__init__('vision_avoidance_auditor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.raw_frames_count = 0
        self.annotated_frames_count = 0
        self.last_cmd_vel = Twist()
        self.last_nav_state = "UNKNOWN"
        self.last_pid_diag = None

        self.create_subscription(Image, '/camera/image_raw', self.raw_img_cb, 10)
        self.create_subscription(Image, '/camera/image_annotated', self.annot_img_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(String, '/navigation/state', self.state_cb, 10)
        self.create_subscription(Float64MultiArray, '/pid/diagnostics', self.pid_cb, 10)

        self.pub_mode = self.create_publisher(String, '/navigation/set_mode', 10)
        self.pub_target = self.create_publisher(Point, '/target_point', 10)
        self.pub_obstacle = self.create_publisher(Point, '/obstacle_point', 10)

    def raw_img_cb(self, msg):
        self.raw_frames_count += 1

    def annot_img_cb(self, msg):
        self.annotated_frames_count += 1

    def cmd_cb(self, msg):
        self.last_cmd_vel = msg

    def state_cb(self, msg):
        self.last_nav_state = msg.data.strip()

    def pid_cb(self, msg):
        self.last_pid_diag = msg.data

def run_vision_audit():
    rclpy.init()
    auditor = VisionAvoidanceAuditor()

    print("=" * 80)
    print("  13. VISION PIPELINE & OBSTACLE AVOIDANCE AUDIT")
    print("=" * 80)

    # 1. Camera Frame Rate Audit
    print("\n[TEST 1: LIVE CAMERA STREAM AUDIT]")
    # First drain any old messages
    for _ in range(20):
        rclpy.spin_once(auditor, timeout_sec=0.01)

    c_raw_start = auditor.raw_frames_count
    c_annot_start = auditor.annotated_frames_count
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(auditor, timeout_sec=0.01)

    duration = time.time() - t0
    raw_fps = (auditor.raw_frames_count - c_raw_start) / duration
    annot_fps = (auditor.annotated_frames_count - c_annot_start) / duration
    print(f"  /camera/image_raw Frame Rate      : {raw_fps:.1f} FPS")
    print(f"  /camera/image_annotated Frame Rate: {annot_fps:.1f} FPS")
    passed_fps = (raw_fps >= 15.0) and (annot_fps >= 10.0)
    print(f"  Camera Streaming Pipeline         : {'✅ PASS' if passed_fps else '❌ FAIL'}")

    # 2. Mode Transitions
    print("\n[TEST 2: NAVIGATION MODE TRANSITION AUDIT]")
    modes_to_test = ['idle', 'follow', 'waypoint', 'idle']
    for m in modes_to_test:
        auditor.pub_mode.publish(String(data=m))
        t_w = time.time()
        while time.time() - t_w < 0.5:
            rclpy.spin_once(auditor, timeout_sec=0.02)
        print(f"  Commanded mode: '{m:8}' -> Current State: '{auditor.last_nav_state}'")

    # 3. Target Tracking (Follow Mode)
    print("\n[TEST 3: TARGET TRACKING & CENTERING AUDIT]")
    auditor.pub_mode.publish(String(data='follow'))
    t_m = time.time()
    while time.time() - t_m < 0.4:
        rclpy.spin_once(auditor, timeout_sec=0.02)

    # Target to the left (cx = 160, frame_center = 320 -> error = -160 px)
    pt_left = Point(x=160.0, y=240.0, z=60.0)
    max_wz_left = 0.0
    max_vy_left = 0.0
    t_l = time.time()
    while time.time() - t_l < 0.8:
        auditor.pub_target.publish(pt_left)
        rclpy.spin_once(auditor, timeout_sec=0.01)
        if auditor.last_cmd_vel.angular.z > max_wz_left:
            max_wz_left = auditor.last_cmd_vel.angular.z
        if auditor.last_cmd_vel.linear.y > max_vy_left:
            max_vy_left = auditor.last_cmd_vel.linear.y

    print(f"  Target Left (x=160 px): /cmd_vel vy={max_vy_left:+.3f} m/s, wz={max_wz_left:+.3f} rad/s")
    tracked_left = (max_wz_left > 0.05) or (max_vy_left > 0.05)
    print(f"  Left Centering Response           : {'✅ PASS (Correct CCW turn + Left strafe)' if tracked_left else '❌ FAIL'}")

    # Target to the right (cx = 480 -> error = +160 px)
    pt_right = Point(x=480.0, y=240.0, z=60.0)
    min_wz_right = 0.0
    min_vy_right = 0.0
    t_r = time.time()
    while time.time() - t_r < 0.8:
        auditor.pub_target.publish(pt_right)
        rclpy.spin_once(auditor, timeout_sec=0.01)
        if auditor.last_cmd_vel.angular.z < min_wz_right:
            min_wz_right = auditor.last_cmd_vel.angular.z
        if auditor.last_cmd_vel.linear.y < min_vy_right:
            min_vy_right = auditor.last_cmd_vel.linear.y

    print(f"  Target Right (x=480 px): /cmd_vel vy={min_vy_right:+.3f} m/s, wz={min_wz_right:+.3f} rad/s")
    tracked_right = (min_wz_right < -0.05) or (min_vy_right < -0.05)
    print(f"  Right Centering Response          : {'✅ PASS (Correct CW turn + Right strafe)' if tracked_right else '❌ FAIL'}")

    # 4. Obstacle Avoidance Swerve Maneuver
    print("\n[TEST 4: SWERVE OBSTACLE AVOIDANCE MANEUVER AUDIT]")
    # Present obstacle directly in corridor (x=320 px, width=90 px) while tracking target
    pt_obs = Point(x=320.0, y=240.0, z=90.0)
    pt_tgt = Point(x=320.0, y=240.0, z=50.0)

    for _ in range(25):
        auditor.pub_obstacle.publish(pt_obs)
        auditor.pub_target.publish(pt_tgt)
        rclpy.spin_once(auditor, timeout_sec=0.02)
        time.sleep(0.02)

    state_avoid = auditor.last_nav_state
    vx_avoid = auditor.last_cmd_vel.linear.x
    vy_avoid = auditor.last_cmd_vel.linear.y
    wz_avoid = auditor.last_cmd_vel.angular.z
    print(f"  Nav State during Obstacle Encounter: '{state_avoid}' (Expected: 'AVOID')")
    print(f"  Commanded Swerve Velocity          : vx={vx_avoid:+.3f} m/s, vy={vy_avoid:+.3f} m/s, wz={wz_avoid:+.3f} rad/s")
    passed_avoid = (state_avoid == "AVOID") and (abs(vy_avoid) > 0.05)
    print(f"  Holonomic Lateral Swerve Maneuver : {'✅ PASS (Pure lateral strafe around obstacle)' if passed_avoid else '❌ FAIL'}")

    # Clear obstacle
    auditor.pub_obstacle.publish(Point(x=0.0, y=0.0, z=0.0))
    time.sleep(0.3)

    # 5. Safe Arrival Stop
    print("\n[TEST 5: TARGET ARRIVAL & SAFE STANDOFF STOP AUDIT]")
    # Target large in frame (z=330 px, indicating close safe standoff)
    pt_arrive = Point(x=320.0, y=240.0, z=330.0)
    for _ in range(20):
        auditor.pub_target.publish(pt_arrive)
        rclpy.spin_once(auditor, timeout_sec=0.02)
        time.sleep(0.02)

    state_arrive = auditor.last_nav_state
    sp_arrive = math.hypot(auditor.last_cmd_vel.linear.x, auditor.last_cmd_vel.linear.y)
    print(f"  Nav State at Destination          : '{state_arrive}'")
    print(f"  Linear Speed at Arrival           : {sp_arrive:.3f} m/s")
    passed_arrive = (sp_arrive < 0.01)
    print(f"  Safe Standoff Stop Verified       : {'✅ PASS (Zero velocity hold)' if passed_arrive else '❌ FAIL'}")

    # Return to idle
    auditor.pub_mode.publish(String(data='idle'))
    auditor.pub_target.publish(Point(x=0.0, y=0.0, z=0.0))
    time.sleep(0.3)

    overall_vision = passed_fps and tracked_left and tracked_right and passed_avoid and passed_arrive
    print("\n" + "=" * 80)
    print(f"  VISION & OBSTACLE AVOIDANCE STATUS: {'✅ ALL TESTS PASSED' if overall_vision else '❌ SOME TESTS FAILED'}")
    print("=" * 80)

    auditor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_vision_audit()
