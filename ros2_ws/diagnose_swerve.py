#!/usr/bin/env python3
"""
diagnose_swerve.py -- Controlled 5-test suite for diagnosing swerve motion accuracy in Gazebo.
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class SwerveDiagnosticNode(Node):
    def __init__(self):
        super().__init__('swerve_diagnostic_node')

        self.current_odom = None
        self.joint_positions = {}
        self.joint_velocities = {}
        self.steer_cmds = {}
        self.wheel_cmds = {}

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Helper subscriptions
        self.create_subscription(Float64, '/fl_steer_cmd', lambda msg: self.steer_cmd_cb('fl', msg.data), 10)
        self.create_subscription(Float64, '/fr_steer_cmd', lambda msg: self.steer_cmd_cb('fr', msg.data), 10)
        self.create_subscription(Float64, '/rl_steer_cmd', lambda msg: self.steer_cmd_cb('rl', msg.data), 10)
        self.create_subscription(Float64, '/rr_steer_cmd', lambda msg: self.steer_cmd_cb('rr', msg.data), 10)

        self.create_subscription(Float64, '/fl_wheel_cmd', lambda msg: self.wheel_cmd_cb('fl', msg.data), 10)
        self.create_subscription(Float64, '/fr_wheel_cmd', lambda msg: self.wheel_cmd_cb('fr', msg.data), 10)
        self.create_subscription(Float64, '/rl_wheel_cmd', lambda msg: self.wheel_cmd_cb('rl', msg.data), 10)
        self.create_subscription(Float64, '/rr_wheel_cmd', lambda msg: self.wheel_cmd_cb('rr', msg.data), 10)

    def odom_cb(self, msg: Odometry):
        self.current_odom = msg

    def joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def steer_cmd_cb(self, mod, val):
        self.steer_cmds[mod] = val

    def wheel_cmd_cb(self, mod, val):
        self.wheel_cmds[mod] = val

    def get_pose(self):
        if self.current_odom is None:
            return 0.0, 0.0, 0.0
        p = self.current_odom.pose.pose.position
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return p.x, p.y, yaw

    def run_test(self, test_name: str, vx: float, vy: float, wz: float, duration: float = 3.0):
        print(f"\n=======================================================")
        print(f"RUNNING {test_name}: vx={vx:+.2f}, vy={vy:+.2f}, wz={wz:+.2f}")
        print(f"=======================================================")

        # Wait for odom
        t_wait = time.time()
        while self.current_odom is None and time.time() - t_wait < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        x0, y0, yaw0 = self.get_pose()
        print(f"Start Pose: x={x0:+.3f}m, y={y0:+.3f}m, yaw={math.degrees(yaw0):+.1f}°")

        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)

        active_steer_cmds = {}
        active_wheel_cmds = {}
        active_steer_feedback = {}
        active_wheel_feedback = {}

        t_start = time.time()
        while time.time() - t_start < duration:
            self.pub_cmd_vel.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            # Sample midpoint
            if time.time() - t_start > (duration / 2.0) and not active_steer_cmds:
                active_steer_cmds = dict(self.steer_cmds)
                active_wheel_cmds = dict(self.wheel_cmds)
                for m in ['fl', 'fr', 'rl', 'rr']:
                    active_steer_feedback[m] = self.joint_positions.get(f"{m}_steer_joint", 0.0)
                    active_wheel_feedback[m] = self.joint_velocities.get(f"{m}_wheel_joint", 0.0)
            time.sleep(0.05)

        # Stop
        self.pub_cmd_vel.publish(Twist())
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        x1, y1, yaw1 = self.get_pose()
        dx = x1 - x0
        dy = y1 - y0
        dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))

        print(f"End Pose  : x={x1:+.3f}m, y={y1:+.3f}m, yaw={math.degrees(yaw1):+.1f}°")
        print(f"Displacement: dx={dx:+.3f}m, dy={dy:+.3f}m, dyaw={math.degrees(dyaw):+.1f}°")
        print("Active Commanded Joint Angles (deg) & Speeds (rad/s):")
        for m in ['fl', 'fr', 'rl', 'rr']:
            st_deg = math.degrees(active_steer_cmds.get(m, 0.0))
            wh_spd = active_wheel_cmds.get(m, 0.0)
            print(f"  {m.upper()}: steer={st_deg:6.1f}°, wheel={wh_spd:6.2f} rad/s")

        print("Active Feedback Joint Angles (deg) & Speeds (rad/s):")
        for m in ['fl', 'fr', 'rl', 'rr']:
            pos_deg = math.degrees(active_steer_feedback.get(m, 0.0))
            vel = active_wheel_feedback.get(m, 0.0)
            print(f"  {m.upper()}: steer={pos_deg:6.1f}°, wheel={vel:6.2f} rad/s")

        return {
            'dx': dx,
            'dy': dy,
            'dyaw': math.degrees(dyaw),
            'steer_cmds': active_steer_cmds,
            'wheel_cmds': active_wheel_cmds
        }

def main():
    rclpy.init()
    diag = SwerveDiagnosticNode()

    # TEST 1: Pure Forward
    t1 = diag.run_test("TEST 1 — Pure forward", vx=0.35, vy=0.0, wz=0.0, duration=3.0)

    # TEST 2: Pure Backward
    t2 = diag.run_test("TEST 2 — Pure backward", vx=-0.35, vy=0.0, wz=0.0, duration=3.0)

    # TEST 3: Pure Lateral (Left)
    t3 = diag.run_test("TEST 3 — Pure lateral (Left)", vx=0.0, vy=0.35, wz=0.0, duration=3.0)

    # TEST 4: Opposite Lateral (Right)
    t4 = diag.run_test("TEST 4 — Opposite lateral (Right)", vx=0.0, vy=-0.35, wz=0.0, duration=3.0)

    # TEST 5: Pure Rotation (CCW)
    t5 = diag.run_test("TEST 5 — Pure rotation (CCW)", vx=0.0, vy=0.0, wz=0.50, duration=3.0)

    diag.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
