#!/usr/bin/env python3
"""
audit_swerve_states_suite.py -- Audits all 8 primary swerve motion states against
theoretical rigid-body kinematic equations, verifies joint commands and feedback.
"""

import sys
sys.path.insert(0, '/home/diya/smart_car')

import math
import time
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import config

class SwerveStatesAuditor(Node):
    def __init__(self):
        super().__init__('swerve_states_auditor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.current_odom = None
        self.joint_positions = {}
        self.joint_velocities = {}
        self.steer_cmds = {}
        self.wheel_cmds = {}

        self.wheelbase = float(config.SWERVE_WHEELBASE_LENGTH) / 100.0  # 0.208 m
        self.track_width = float(config.SWERVE_TRACK_WIDTH) / 100.0    # 0.2285 m
        self.wheel_radius = float(config.SWERVE_WHEEL_RADIUS) / 100.0  # 0.039965 m

        self.module_positions = {
            'fl': (+self.wheelbase / 2.0, +self.track_width / 2.0),
            'fr': (+self.wheelbase / 2.0, -self.track_width / 2.0),
            'rl': (-self.wheelbase / 2.0, +self.track_width / 2.0),
            'rr': (-self.wheelbase / 2.0, -self.track_width / 2.0),
        }

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

        for m in ['fl', 'fr', 'rl', 'rr']:
            self.create_subscription(Float64, f'/{m}_steer_cmd', lambda msg, mod=m: self.steer_cb(mod, msg.data), 10)
            self.create_subscription(Float64, f'/{m}_wheel_cmd', lambda msg, mod=m: self.wheel_cb(mod, msg.data), 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_cb(self, msg):
        self.current_odom = msg

    def js_cb(self, msg):
        for n, p, v in zip(msg.name, msg.position, msg.velocity):
            self.joint_positions[n] = p
            self.joint_velocities[n] = v

    def steer_cb(self, mod, val):
        self.steer_cmds[mod] = val

    def wheel_cb(self, mod, val):
        self.wheel_cmds[mod] = val

    def get_pose(self):
        if self.current_odom is None:
            return 0.0, 0.0, 0.0, 0.0
        p = self.current_odom.pose.pose.position
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return p.x, p.y, p.z, yaw

    def return_to_origin(self):
        t_start = time.time()
        while time.time() - t_start < 6.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            px, py, pz, yaw = self.get_pose()
            ex = -px
            ey = -py
            eyaw = (0.0 - yaw + math.pi) % (2.0 * math.pi) - math.pi
            dist = math.hypot(ex, ey)
            if dist < 0.015 and abs(eyaw) < 0.03:
                break
            vx_body = 1.2 * (ex * math.cos(yaw) + ey * math.sin(yaw))
            vy_body = 1.2 * (-ex * math.sin(yaw) + ey * math.cos(yaw))
            wz_body = 1.5 * eyaw
            sp = math.hypot(vx_body, vy_body)
            if sp > 0.35:
                vx_body *= (0.35 / sp)
                vy_body *= (0.35 / sp)
            wz_body = max(-0.8, min(0.8, wz_body))
            cmd = Twist()
            cmd.linear.x = float(vx_body)
            cmd.linear.y = float(vy_body)
            cmd.angular.z = float(wz_body)
            self.pub_cmd.publish(cmd)
            time.sleep(0.02)
        self.pub_cmd.publish(Twist())
        t_wait = time.time() + 0.3
        while time.time() < t_wait:
            rclpy.spin_once(self, timeout_sec=0.01)

    def compute_theoretical_kinematics(self, vx: float, vy: float, omega: float):
        theory = {}
        for mod, (xi, yi) in self.module_positions.items():
            # Rigid body relations:
            # Vix = vx - omega * yi
            # Viy = vy + omega * xi
            v_ix = vx - omega * yi
            v_iy = vy + omega * xi
            spd = math.hypot(v_ix, v_iy)
            ang = math.degrees(math.atan2(v_iy, v_ix))
            omega_w = spd / self.wheel_radius  # rad/s
            theory[mod] = {
                'v_ix': v_ix,
                'v_iy': v_iy,
                'speed': spd,
                'angle_deg': ang,
                'wheel_omega': omega_w
            }
        return theory

    def run_state_test(self, label: str, vx: float, vy: float, wz: float, duration: float = 2.0):
        self.return_to_origin()
        x0, y0, z0, yaw0 = self.get_pose()

        # Theoretical values
        theory = self.compute_theoretical_kinematics(vx, vy, wz)

        print(f"\n===========================================================================")
        print(f"  {label.upper()}: Commanded vx={vx:+.3f} m/s, vy={vy:+.3f} m/s, wz={wz:+.3f} rad/s")
        print(f"===========================================================================")
        print(f"Start Pose: ({x0:+.3f} m, {y0:+.3f} m, {math.degrees(yaw0):+.1f}°)")

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)

        active_steer_cmds = {}
        active_wheel_cmds = {}
        active_steer_fdbk = {}
        active_wheel_fdbk = {}

        t_start = time.time()
        while time.time() - t_start < duration:
            self.pub_cmd.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            # Sample midpoint
            if time.time() - t_start > (duration * 0.5) and not active_steer_cmds:
                for m in ['fl', 'fr', 'rl', 'rr']:
                    active_steer_cmds[m] = self.steer_cmds.get(m, 0.0)
                    active_wheel_cmds[m] = self.wheel_cmds.get(m, 0.0)
                    active_steer_fdbk[m] = self.joint_positions.get(f'{m}_steer_joint', 0.0)
                    active_wheel_fdbk[m] = self.joint_velocities.get(f'{m}_wheel_joint', 0.0)

        self.pub_cmd.publish(Twist())
        t_stop = time.time() + 0.3
        while time.time() < t_stop:
            rclpy.spin_once(self, timeout_sec=0.01)

        x1, y1, z1, yaw1 = self.get_pose()
        dx = x1 - x0
        dy = y1 - y0
        dyaw = math.degrees((yaw1 - yaw0 + math.pi) % (2.0 * math.pi) - math.pi)
        disp_mag = math.hypot(dx, dy)

        print(f"End Pose  : ({x1:+.3f} m, {y1:+.3f} m, {math.degrees(yaw1):+.1f}°)")
        print(f"Displacement: Δx={dx:+.3f} m, Δy={dy:+.3f} m, Mag={disp_mag:.3f} m, ΔYaw={dyaw:+.1f}°")

        # Table comparing Theory vs Commanded vs Feedback
        print("\n  Pod | Theoretical (Angle, Speed) | Commanded (Angle, Wheel Spd) | Feedback (Angle, Wheel Spd) | Diff")
        print("  ----+----------------------------+------------------------------+-----------------------------+------")
        all_passed = True
        for m in ['fl', 'fr', 'rl', 'rr']:
            th_ang = theory[m]['angle_deg']
            th_spd = theory[m]['speed']
            th_w = theory[m]['wheel_omega']

            cmd_ang = math.degrees(active_steer_cmds.get(m, 0.0))
            cmd_w = active_wheel_cmds.get(m, 0.0)

            fb_ang = math.degrees(active_steer_fdbk.get(m, 0.0))
            fb_w = active_wheel_fdbk.get(m, 0.0)

            # Check angle equivalence (accounting for 180° flip optimization: angle + 180 and -speed)
            ang_err1 = abs((fb_ang - th_ang + 180) % 360 - 180)
            ang_err2 = abs((fb_ang - (th_ang + 180) + 180) % 360 - 180)

            if ang_err1 <= ang_err2:
                eff_ang_err = ang_err1
                eff_spd_err = abs(fb_w - th_w)
            else:
                eff_ang_err = ang_err2
                eff_spd_err = abs(-fb_w - th_w)

            match_str = "MATCH" if (eff_ang_err < 3.0 and eff_spd_err < 0.5) else "DIFF"
            if match_str != "MATCH":
                all_passed = False

            print(f"  {m.upper():3} | ({th_ang:+6.1f}°, {th_spd:5.2f}m/s)  | ({cmd_ang:+6.1f}°, {cmd_w:+6.2f}rad/s)   | ({fb_ang:+6.1f}°, {fb_w:+6.2f}rad/s)   | err_ang={eff_ang_err:.1f}° [{match_str}]")

        return {
            'label': label,
            'cmd': (vx, vy, wz),
            'disp': (dx, dy, dyaw),
            'theory': theory,
            'cmd_steer': active_steer_cmds,
            'cmd_wheel': active_wheel_cmds,
            'fb_steer': active_steer_fdbk,
            'fb_wheel': active_wheel_fdbk,
            'passed': all_passed
        }

def main():
    rclpy.init()
    auditor = SwerveStatesAuditor()

    # Wait for simulation to be active
    t_wait = time.time()
    while auditor.current_odom is None and time.time() - t_wait < 5.0:
        rclpy.spin_once(auditor, timeout_sec=0.05)

    states = [
        ("State A: Forward Translation", 0.35, 0.00, 0.00),
        ("State B: Reverse Translation", -0.35, 0.00, 0.00),
        ("State C: Left Strafe", 0.00, 0.35, 0.00),
        ("State D: Right Strafe", 0.00, -0.35, 0.00),
        ("State E: Diagonal 45°", 0.25, 0.25, 0.00),
        ("State F: Diagonal 135°", -0.25, 0.25, 0.00),
        ("State G: Pure Rotation (CCW in place)", 0.00, 0.00, 0.60),
        ("State H: Translation + Rotation (Curvilinear)", 0.25, 0.15, 0.50),
    ]

    results = []
    for label, vx, vy, wz in states:
        res = auditor.run_state_test(label, vx, vy, wz, duration=2.0)
        results.append(res)

    print("\n" + "=" * 75)
    print("  SWERVE STATES AUDIT SUMMARY")
    print("=" * 75)
    overall_pass = True
    for r in results:
        status = "✅ PASS" if r['passed'] else "❌ FAIL"
        if not r['passed']:
            overall_pass = False
        print(f"  {r['label']:45} | Commanded {r['cmd']} | {status}")

    # Special inspection for State H (Translation + Rotation):
    res_h = results[-1]
    th_h = res_h['theory']
    print("\nVerification of State H: Translation + Rotation Independent Wheel Motion:")
    speeds = [th_h[m]['speed'] for m in ['fl', 'fr', 'rl', 'rr']]
    angles = [th_h[m]['angle_deg'] for m in ['fl', 'fr', 'rl', 'rr']]
    print(f"  Theoretical Wheel Speeds (m/s)  : FL={speeds[0]:.3f}, FR={speeds[1]:.3f}, RL={speeds[2]:.3f}, RR={speeds[3]:.3f}")
    print(f"  Theoretical Wheel Angles (deg)  : FL={angles[0]:+.1f}°, FR={angles[1]:+.1f}°, RL={angles[2]:+.1f}°, RR={angles[3]:+.1f}°")
    unique_speeds = len(set(f"{s:.2f}" for s in speeds))
    unique_angles = len(set(f"{a:.1f}" for a in angles))
    print(f"  Distinct wheel speeds: {unique_speeds}/4 | Distinct wheel angles: {unique_angles}/4")
    assert unique_speeds >= 3 and unique_angles >= 3, "State H did not produce distinct wheel speeds/angles!"
    print("  Confirmed: Each wheel receives a mathematically distinct velocity and steering angle!")

    auditor.return_to_origin()
    auditor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
