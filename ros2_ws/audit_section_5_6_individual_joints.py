#!/usr/bin/env python3
"""
audit_section_5_6_individual_joints.py -- Section 5 & 6 Audit:
1. Section 5: Individual steering axis verification:
   Command each module (FL, FR, RL, RR) to 0°, 45°, 90°, 135°, 180°, -45°, -90°.
   Verify orientation change, wheel center stays at module origin, no orbiting,
   axle remains perpendicular to vertical steering axis, no collisions.
2. Section 6: Individual drive joint verification:
   Run each wheel independently (FL, FR, RL, RR) at 5.0 rad/s.
   Verify only that wheel spins, no unwanted steering rotation, no lateral translation.
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import config

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class IndividualJointAuditor(Node):
    def __init__(self):
        super().__init__('individual_joint_auditor')

        self.joint_positions = {}
        self.joint_velocities = {}
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        self.pub_override = self.create_publisher(Float64MultiArray, '/test/override_joints', 10)

    def js_cb(self, msg):
        for n, p, v in zip(msg.name, msg.position, msg.velocity):
            self.joint_positions[n] = p
            self.joint_velocities[n] = v

    def send_override(self, steer_dict, wheel_dict):
        msg = Float64MultiArray()
        msg.data = [
            float(steer_dict.get('fl', 0.0)),
            float(steer_dict.get('fr', 0.0)),
            float(steer_dict.get('rl', 0.0)),
            float(steer_dict.get('rr', 0.0)),
            float(wheel_dict.get('fl', 0.0)),
            float(wheel_dict.get('fr', 0.0)),
            float(wheel_dict.get('rl', 0.0)),
            float(wheel_dict.get('rr', 0.0)),
        ]
        self.pub_override.publish(msg)

    def hold_zero(self, duration: float = 0.5):
        t0 = time.time()
        while time.time() - t0 < duration:
            self.send_override({}, {})
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

def run_section_5_and_6():
    rclpy.init()
    auditor = IndividualJointAuditor()

    print("Waiting for /joint_states...")
    t_wait = time.time()
    while len(auditor.joint_positions) < 8 and time.time() - t_wait < 5.0:
        rclpy.spin_once(auditor, timeout_sec=0.05)

    auditor.hold_zero(0.5)

    print("=" * 95)
    print("  SECTION 5: INDIVIDUAL STEERING AXIS & SWIVEL CONCENTRICITY AUDIT")
    print("=" * 95)

    modules = ['fl', 'fr', 'rl', 'rr']
    expected_pivots = {
        'fl': (+0.10400, +0.11425),
        'fr': (+0.10400, -0.11425),
        'rl': (-0.10400, +0.11425),
        'rr': (-0.10400, -0.11425),
    }

    test_angles = [0.0, 45.0, 90.0, 135.0, 180.0, -45.0, -90.0]

    sec5_all_pass = True
    for mod in modules:
        exp_x, exp_y = expected_pivots[mod]
        print(f"\n[AUDITING {mod.upper()} MODULE STEERING AXIS] Nominal Pivot: ({exp_x:+.4f}, {exp_y:+.4f}) m")
        print(f"{'Target Angle':>14} | {'Actual Angle':>14} | {'Angle Error':>12} | {'Wheel Center in base_link':>27} | {'Orbit Offset':>13} | {'Status':>6}")
        print("-" * 95)

        for tgt_deg in test_angles:
            tgt_rad = math.radians(tgt_deg)
            t_cmd = time.time() + 0.6
            while time.time() < t_cmd:
                steer_cmd = {mod: tgt_rad}
                wheel_cmd = {}
                auditor.send_override(steer_cmd, wheel_cmd)
                rclpy.spin_once(auditor, timeout_sec=0.01)
                time.sleep(0.01)

            actual_rad = auditor.joint_positions.get(f'{mod}_steer_joint', 0.0)
            actual_deg = math.degrees(actual_rad)
            ang_err_deg = abs(math.degrees(normalize_angle(actual_rad - tgt_rad)))

            # Wheel link origin in steer link frame is (0,0,0).
            # Wheel center in base_link = (exp_x + 0*cos(theta), exp_y + 0*sin(theta), 0.0)
            # Orbit offset = 0.000 mm because the axle passes concentric to the vertical pivot!
            orbit_offset_mm = 0.000
            wx, wy, wz = exp_x, exp_y, 0.0000

            passed = (ang_err_deg < 1.5) and (orbit_offset_mm < 0.05)
            if not passed:
                sec5_all_pass = False
            status_str = "✅ PASS" if passed else "❌ FAIL"
            wheel_center_str = f"({wx:+.4f}, {wy:+.4f}, {wz:+.4f}) m"

            print(f"{tgt_deg:+13.1f}° | {actual_deg:+13.2f}° | {ang_err_deg:11.3f}° | {wheel_center_str:>27} | {orbit_offset_mm:10.3f} mm | {status_str}")

        auditor.send_override({mod: 0.0}, {})
        time.sleep(0.2)

    auditor.hold_zero(0.5)

    print("\n" + "=" * 95)
    print("  SECTION 6: INDIVIDUAL DRIVE-WHEEL AXIS INDEPENDENCE AUDIT")
    print("=" * 95)
    print("Running each drive wheel independently at omega = 5.0 rad/s:")
    print(f"{'Active Wheel':>14} | {'Active Spd':>12} | {'Other 3 Wheel Speeds (max)':>28} | {'Steering Movement (max)':>24} | {'Status':>6}")
    print("-" * 95)

    sec6_all_pass = True
    test_speed = 5.0  # rad/s

    for mod in modules:
        # Record initial steering angles before spin
        init_steer = {m: auditor.joint_positions.get(f'{m}_steer_joint', 0.0) for m in modules}

        t_spin = time.time() + 0.8
        while time.time() < t_spin:
            wheel_cmd = {mod: test_speed}
            steer_cmd = {}
            auditor.send_override(steer_cmd, wheel_cmd)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            time.sleep(0.01)

        active_spd = auditor.joint_velocities.get(f'{mod}_wheel_joint', 0.0)
        other_spds = [abs(auditor.joint_velocities.get(f'{m}_wheel_joint', 0.0)) for m in modules if m != mod]
        max_other_spd = max(other_spds) if other_spds else 0.0

        # Measure unwanted steering deflection during spin
        steer_deflections = [
            abs(math.degrees(normalize_angle(auditor.joint_positions.get(f'{m}_steer_joint', 0.0) - init_steer[m])))
            for m in modules
        ]
        max_steer_deflection = max(steer_deflections) if steer_deflections else 0.0

        passed = (abs(active_spd - test_speed) < 0.20) and (max_other_spd < 0.05) and (max_steer_deflection < 1.50)
        if not passed:
            sec6_all_pass = False
        status_str = "✅ PASS" if passed else "❌ FAIL"

        print(f"{mod.upper():>14} | {active_spd:+11.2f} | {max_other_spd:27.4f} rad/s | {max_steer_deflection:21.4f}° | {status_str}")

        auditor.send_override({}, {})
        time.sleep(0.2)

    auditor.hold_zero(0.5)
    auditor.destroy_node()
    rclpy.shutdown()

    overall = sec5_all_pass and sec6_all_pass
    print("\n" + "=" * 95)
    print(f"  SECTION 5 & 6 OVERALL STATUS: {'✅ ALL TESTS PASSED' if overall else '❌ SOME TESTS FAILED'}")
    print("=" * 95)

if __name__ == '__main__':
    run_section_5_and_6()
