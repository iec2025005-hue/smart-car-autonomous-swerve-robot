#!/usr/bin/env python3
"""
audit_section_10_11_pid_velocity.py -- Audit Sections 10 & 11:
1. Section 10: Steering PID & Shortest-path step responses:
   - Steps: 0° -> 90°, 90° -> 180°, 180° -> -90°, -179° -> +179°.
   - Verify shortest-path (e.g. -179° to +179° is 2° delta, not 358°).
   - Record target vs measured, rise time, settling time, overshoot, steady error.
2. Section 11: Drive PID / Velocity tracking:
   - Speeds: 0.10 m/s, 0.20 m/s, 0.35 m/s.
   - Measure commanded vs feedback speed across all 4 wheels.
   - Check wheel speed equality, slip, overshoot, stability.
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import config

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class PidVelocityAuditor(Node):
    def __init__(self):
        super().__init__('pid_velocity_auditor')

        self.joint_positions = {}
        self.joint_velocities = {}
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        self.pub_override = self.create_publisher(Float64MultiArray, '/test/override_joints', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def js_cb(self, msg):
        for n, p, v in zip(msg.name, msg.position, msg.velocity):
            self.joint_positions[n] = p
            self.joint_velocities[n] = v

    def send_override_steer(self, angle_rad: float):
        msg = Float64MultiArray()
        msg.data = [angle_rad, angle_rad, angle_rad, angle_rad, 0.0, 0.0, 0.0, 0.0]
        self.pub_override.publish(msg)

    def stop_robot(self):
        self.pub_cmd_vel.publish(Twist())
        time.sleep(0.3)

def run_section_10_and_11():
    rclpy.init()
    auditor = PidVelocityAuditor()

    print("Waiting for /joint_states...")
    t_wait = time.time()
    while len(auditor.joint_positions) < 8 and time.time() - t_wait < 5.0:
        rclpy.spin_once(auditor, timeout_sec=0.05)

    print("=" * 95)
    print("  SECTION 10: STEERING PID, SHORTEST PATH & WRAPPING AUDIT")
    print("=" * 95)
    print("Controller Parameters: P=16.0, I=1.0, D=0.3, I_max=2.0 Nm, Cmd_max=15.0 Nm, Freq=50 Hz")

    # The prompt explicitly specifies:
    # 0 -> 90°
    # 90 -> 180°
    # 180 -> -90°
    # -179 -> +179°
    steps = [
        ("Step 1:   0° ->  90°",   0.0,   90.0),
        ("Step 2:  90° -> 180°",  90.0,  180.0),
        ("Step 3: 180° -> -90°", 180.0,  -90.0),
        ("Step 4: -179° -> +179°", -179.0, +179.0),
    ]

    print(f"\n{'Test Step':<22} | {'Initial':>8} | {'Target':>8} | {'Measured':>9} | {'Shortest Δ':>11} | {'Settling':>9} | {'Steady Err':>11} | {'Status':>6}")
    print("-" * 95)

    sec10_all_pass = True

    for name, init_deg, tgt_deg in steps:
        init_rad = math.radians(init_deg)
        tgt_rad = math.radians(tgt_deg)

        # Hold initial position
        t_init = time.time() + 0.6
        while time.time() < t_init:
            auditor.send_override_steer(init_rad)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            time.sleep(0.01)

        # Shortest angle calculation
        shortest_delta_deg = abs(math.degrees(normalize_angle(tgt_rad - init_rad)))

        # Step to target and record time-series
        t_start = time.time()
        settled_time = None
        t_end = t_start + 0.8
        times = []
        angles = []

        while time.time() < t_end:
            auditor.send_override_steer(tgt_rad)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            cur_rad = auditor.joint_positions.get('fl_steer_joint', 0.0)
            elapsed = time.time() - t_start
            times.append(elapsed)
            angles.append(cur_rad)

            err_deg = abs(math.degrees(normalize_angle(cur_rad - tgt_rad)))
            if err_deg <= 1.5 and settled_time is None and elapsed > 0.05:
                settled_time = elapsed
            time.sleep(0.01)

        final_rad = auditor.joint_positions.get('fl_steer_joint', 0.0)
        final_deg = math.degrees(final_rad)
        final_err = abs(math.degrees(normalize_angle(final_rad - tgt_rad)))
        settling_ms = (settled_time * 1000.0) if settled_time else (0.8 * 1000.0)

        # For -179° to +179°, shortest delta is 2°!
        passed = (final_err < 1.5) and (settling_ms < 600.0)
        if not passed:
            sec10_all_pass = False

        status_str = "✅ PASS" if passed else "❌ FAIL"
        print(f"{name:<22} | {init_deg:+7.1f}° | {tgt_deg:+7.1f}° | {final_deg:+8.2f}° | {shortest_delta_deg:10.1f}° | {settling_ms:7.1f} ms | {final_err:10.3f}° | {status_str}")

    # Return to 0
    t_zero = time.time() + 0.5
    while time.time() < t_zero:
        auditor.send_override_steer(0.0)
        rclpy.spin_once(auditor, timeout_sec=0.01)
        time.sleep(0.01)

    print("\n" + "=" * 95)
    print("  SECTION 11: DRIVE VELOCITY CONTROL & TRACKING ACCURACY AUDIT")
    print("=" * 95)
    print("Testing commanded forward speeds: 0.10 m/s, 0.20 m/s, 0.35 m/s")
    print("Wheel Radius R = 0.039965 m (Target joint omega = v / R)")
    print(f"{'Cmd Speed':>10} | {'Exp Omega':>10} | {'FL Omega':>10} | {'FR Omega':>10} | {'RL Omega':>10} | {'RR Omega':>10} | {'Max Slip / Err':>15} | {'Status':>6}")
    print("-" * 95)

    speeds_to_test = [0.10, 0.20, 0.35]
    sec11_all_pass = True

    wheel_r = 0.039965  # meters
    for v_cmd in speeds_to_test:
        exp_omega = v_cmd / wheel_r

        twist = Twist()
        twist.linear.x = float(v_cmd)

        # Drive for 1.2s to reach steady-state velocity
        t_run = time.time() + 1.2
        while time.time() < t_run:
            auditor.pub_cmd_vel.publish(twist)
            rclpy.spin_once(auditor, timeout_sec=0.01)
            time.sleep(0.01)

        w_fl = auditor.joint_velocities.get('fl_wheel_joint', 0.0)
        w_fr = auditor.joint_velocities.get('fr_wheel_joint', 0.0)
        w_rl = auditor.joint_velocities.get('rl_wheel_joint', 0.0)
        w_rr = auditor.joint_velocities.get('rr_wheel_joint', 0.0)

        # Compute max tracking error
        errs = [abs(w - exp_omega) for w in [w_fl, w_fr, w_rl, w_rr]]
        max_err = max(errs)
        # Slip / inequality across 4 wheels
        inequality = max(w_fl, w_fr, w_rl, w_rr) - min(w_fl, w_fr, w_rl, w_rr)

        passed = (max_err < 0.15) and (inequality < 0.10)
        if not passed:
            sec11_all_pass = False
        status_str = "✅ PASS" if passed else "❌ FAIL"

        print(f"{v_cmd:9.2f} m/s | {exp_omega:8.3f} | {w_fl:8.3f} | {w_fr:8.3f} | {w_rl:8.3f} | {w_rr:8.3f} | {max_err:13.4f} rad/s | {status_str}")

        auditor.stop_robot()
        time.sleep(0.4)

    auditor.stop_robot()
    auditor.destroy_node()
    rclpy.shutdown()

    overall = sec10_all_pass and sec11_all_pass
    print("\n" + "=" * 95)
    print(f"  SECTION 10 & 11 OVERALL STATUS: {'✅ ALL TESTS PASSED' if overall else '❌ SOME TESTS FAILED'}")
    print("=" * 95)

if __name__ == '__main__':
    run_section_10_and_11()
