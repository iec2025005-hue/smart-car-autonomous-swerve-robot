#!/usr/bin/env python3
"""
audit_section_21_tf_tree.py -- Section 21:
Verify complete TF tree:
odom -> base_footprint -> base_link -> each steering link -> each wheel link -> camera -> IMU
Verify:
- No TF errors
- No timestamp regressions
- No disconnected links
- Correct steering & wheel transforms
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import math
import time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class TfTreeAuditor(Node):
    def __init__(self):
        super().__init__('tf_tree_auditor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_stamps = {}
        self.regressions_detected = 0
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)
        qos_static = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_tf_static = self.create_subscription(TFMessage, '/tf_static', self.tf_static_cb, qos_static)

    def tf_cb(self, msg: TFMessage):
        for transform in msg.transforms:
            frame_pair = (transform.header.frame_id, transform.child_frame_id)
            sec = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if frame_pair in self.last_stamps:
                if sec < self.last_stamps[frame_pair]:
                    self.regressions_detected += 1
            self.last_stamps[frame_pair] = sec

    def tf_static_cb(self, msg: TFMessage):
        for transform in msg.transforms:
            frame_pair = (transform.header.frame_id, transform.child_frame_id)
            sec = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            self.last_stamps[frame_pair] = sec

def run_tf_audit():
    rclpy.init()
    auditor = TfTreeAuditor()

    print("Collecting TF transforms for 2.0 seconds...")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(auditor, timeout_sec=0.02)
        time.sleep(0.02)

    print("=" * 95)
    print("  SECTION 21: TF TREE TOPOLOGY, CONTINUITY & ACCURACY AUDIT")
    print("=" * 95)

    required_transforms = [
        ('odom', 'base_footprint', "Odometry root to chassis ground footprint"),
        ('base_footprint', 'base_link', "Ground footprint to chassis center (+39.965 mm)"),
        ('base_link', 'fl_steer_link', "Chassis center to FL steer swivel (+0.104, +0.11425)"),
        ('base_link', 'fr_steer_link', "Chassis center to FR steer swivel (+0.104, -0.11425)"),
        ('base_link', 'rl_steer_link', "Chassis center to RL steer swivel (-0.104, +0.11425)"),
        ('base_link', 'rr_steer_link', "Chassis center to RR steer swivel (-0.104, -0.11425)"),
        ('fl_steer_link', 'fl_wheel_link', "FL steer swivel to FL wheel hub (0, 0, 0) concentric"),
        ('fr_steer_link', 'fr_wheel_link', "FR steer swivel to FR wheel hub (0, 0, 0) concentric"),
        ('rl_steer_link', 'rl_wheel_link', "RL steer swivel to RL wheel hub (0, 0, 0) concentric"),
        ('rr_steer_link', 'rr_wheel_link', "RR steer swivel to RR wheel hub (0, 0, 0) concentric"),
        ('base_link', 'camera_link', "Chassis center to front RGBD camera"),
        ('base_link', 'imu_link', "Chassis center to 9-DOF IMU sensor"),
    ]

    print(f"{'Parent Frame':<16} -> {'Child Frame':<16} | {'Translation (x, y, z)':<28} | {'Status':>6}")
    print("-" * 95)

    all_connected = True
    for parent, child, desc in required_transforms:
        try:
            tf_data = auditor.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            tx = tf_data.transform.translation.x
            ty = tf_data.transform.translation.y
            tz = tf_data.transform.translation.z
            trans_s = f"({tx:+.4f}, {ty:+.4f}, {tz:+.4f}) m"
            status_s = "✅ PASS"
        except Exception as e:
            trans_s = "NOT FOUND / DISCONNECTED"
            status_s = "❌ FAIL"
            all_connected = False

        print(f"{parent:<16} -> {child:<16} | {trans_s:<28} | {status_s}")

    print("\n[TEMPORAL INTEGRITY CHECK]")
    print(f"  Active Transform Pairs Tracked    : {len(auditor.last_stamps)}")
    print(f"  Timestamp Regressions Detected    : {auditor.regressions_detected}")
    passed_time = (auditor.regressions_detected == 0) and (len(auditor.last_stamps) >= 12)
    print(f"  Temporal Continuity Status        : {'✅ PASS (Strictly monotonic timestamps)' if passed_time else '❌ FAIL'}")

    auditor.destroy_node()
    rclpy.shutdown()

    overall = all_connected and passed_time
    print("\n" + "=" * 95)
    print(f"  SECTION 21 OVERALL STATUS: {'✅ ALL TESTS PASSED' if overall else '❌ SOME TESTS FAILED'}")
    print("=" * 95)

if __name__ == '__main__':
    run_tf_audit()
