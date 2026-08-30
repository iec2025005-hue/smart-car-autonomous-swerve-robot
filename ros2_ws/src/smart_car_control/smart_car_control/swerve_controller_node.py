#!/usr/bin/env python3
"""
smart_car_control/swerve_controller_node.py

ROS 2 Swerve Drive Controller Node
- Wraps existing navigation/swerve_kinematics.py without modifying or duplicating equations.
- Converts /cmd_vel (geometry_msgs/Twist) -> 4 steering joint positions (rad) & 4 wheel velocities (rad/s).
- Implements shortest-path steering angle optimization.
- Computes forward kinematics odometry from joint state feedback and publishes /odom & TF (odom -> base_footprint).
"""

import math
import os
import sys
import numpy as np

# Add repository root to python sys.path to import existing algorithms directly
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

# Import the existing swerve kinematics directly
from navigation.swerve_kinematics import SwerveKinematics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Empty
import tf2_ros


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to Quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class SwerveControllerNode(Node):
    def __init__(self):
        super().__init__('swerve_controller_node')

        # ── Parameters & Dimensions (Real Measured Dimensions) ───
        self.wheel_radius = 0.039965   # 39.965 mm outer radius (79.93 mm outer diameter)
        self.wheel_width = 0.03765     # 37.65 mm width
        self.wheelbase = 0.208         # L = 20.8 cm
        self.track_width = 0.2285      # W = 22.85 cm

        # Module physical offsets (REP-103: +X front, +Y left)
        self.module_positions = {
            'fl': (self.wheelbase / 2.0, self.track_width / 2.0),
            'fr': (self.wheelbase / 2.0, -self.track_width / 2.0),
            'rl': (-self.wheelbase / 2.0, self.track_width / 2.0),
            'rr': (-self.wheelbase / 2.0, -self.track_width / 2.0),
        }

        # Existing kinematics instance
        self.kinematics = SwerveKinematics()

        # Previous commanded steering angles (for shortest-path steering optimization)
        self.last_steer_angles = {
            'fl': 0.0,
            'fr': 0.0,
            'rl': 0.0,
            'rr': 0.0
        }

        # ── Odometry State ─────────────────────────────────────────
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = self.get_clock().now()

        # Joint feedback storage
        self.current_joint_positions = {}
        self.current_joint_velocities = {}

        # ── Publishers ─────────────────────────────────────────────
        # 4× Steering Position Command Publishers
        self.pub_fl_steer = self.create_publisher(Float64, '/fl_steer_cmd', 10)
        self.pub_fr_steer = self.create_publisher(Float64, '/fr_steer_cmd', 10)
        self.pub_rl_steer = self.create_publisher(Float64, '/rl_steer_cmd', 10)
        self.pub_rr_steer = self.create_publisher(Float64, '/rr_steer_cmd', 10)

        # 4× Wheel Velocity Command Publishers
        self.pub_fl_wheel = self.create_publisher(Float64, '/fl_wheel_cmd', 10)
        self.pub_fr_wheel = self.create_publisher(Float64, '/fr_wheel_cmd', 10)
        self.pub_rl_wheel = self.create_publisher(Float64, '/rl_wheel_cmd', 10)
        self.pub_rr_wheel = self.create_publisher(Float64, '/rr_wheel_cmd', 10)

        # Odometry Publisher & TF Broadcaster
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscribers ────────────────────────────────────────────
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.sub_reset_odom = self.create_subscription(
            Empty,
            '/reset_odom',
            self.reset_odom_callback,
            10
        )

        # 50 Hz control & odometry loop timer
        self.timer = self.create_timer(0.02, self.control_loop)

        self.target_twist = Twist()
        self.get_logger().info('Swerve Controller Node Initialized with existing SwerveKinematics.')

    def reset_odom_callback(self, msg: Empty):
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_steer_angles = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        self.get_logger().info('Odometry and steering memory reset to (0.0, 0.0, 0.0).')

    def cmd_vel_callback(self, msg: Twist):
        self.target_twist = msg

    def joint_states_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_joint_velocities[name] = msg.velocity[i]

    def optimize_steering(self, actual_angle: float, target_angle: float, target_speed: float):
        """Shortest path steering optimization with continuous angle unwrapping and cosine scaling."""
        if abs(target_speed) < 1e-4:
            # Idle: maintain current angle to prevent jitter
            return actual_angle, 0.0

        # Shortest-angle difference in [-pi, pi]
        delta = normalize_angle(target_angle - actual_angle)

        if abs(delta) > (math.pi / 2.0 + 1e-3):
            # Reverse wheel direction instead of rotating > 90 deg
            if delta > 0:
                opt_delta = delta - math.pi
            else:
                opt_delta = delta + math.pi
            cmd_angle = actual_angle + opt_delta
            cmd_speed = -target_speed
        else:
            cmd_angle = actual_angle + delta
            cmd_speed = target_speed

        # Cosine velocity scaling: scale drive speed by cos(angular error)
        # Prevents wheel scrub/drag while steering pod is rotating
        ang_err = normalize_angle(cmd_angle - actual_angle)
        scale_factor = max(0.0, math.cos(ang_err))
        cmd_speed *= scale_factor

        return cmd_angle, cmd_speed

    def control_loop(self):
        # 1. READ TARGET TWIST (m/s, rad/s in REP-103)
        vx = self.target_twist.linear.x
        vy = self.target_twist.linear.y
        wz = self.target_twist.angular.z

        # 2. EVALUATE SWERVE KINEMATICS
        # Returns [(a_fr, s_fr), (a_fl, s_fl), (a_rl, s_rl), (a_rr, s_rr)]
        # angle in degrees [-180, 180], speed in m/s
        results = self.kinematics.calculate(vx, vy, wz)

        res_fr = results[0]
        res_fl = results[1]
        res_rl = results[2]
        res_rr = results[3]

        raw_modules = {
            'fr': (math.radians(res_fr[0]), res_fr[1]),
            'fl': (math.radians(res_fl[0]), res_fl[1]),
            'rl': (math.radians(res_rl[0]), res_rl[1]),
            'rr': (math.radians(res_rr[0]), res_rr[1]),
        }

        # 3. APPLY SHORTEST-PATH OPTIMIZATION & CONVERT TO WHEEL ANGULAR VELOCITY
        cmd_steer = {}
        cmd_wheel = {}

        for mod, (target_ang, target_spd) in raw_modules.items():
            s_joint = f"{mod}_steer_joint"
            cur_pos = self.current_joint_positions.get(s_joint, self.last_steer_angles[mod])
            opt_ang, opt_spd = self.optimize_steering(cur_pos, target_ang, target_spd)
            self.last_steer_angles[mod] = opt_ang

            cmd_steer[mod] = opt_ang
            # Wheel angular velocity: omega = v / r
            cmd_wheel[mod] = opt_spd / self.wheel_radius

        # 4. PUBLISH JOINT COMMANDS
        self.pub_fl_steer.publish(Float64(data=float(cmd_steer['fl'])))
        self.pub_fr_steer.publish(Float64(data=float(cmd_steer['fr'])))
        self.pub_rl_steer.publish(Float64(data=float(cmd_steer['rl'])))
        self.pub_rr_steer.publish(Float64(data=float(cmd_steer['rr'])))

        self.pub_fl_wheel.publish(Float64(data=float(cmd_wheel['fl'])))
        self.pub_fr_wheel.publish(Float64(data=float(cmd_wheel['fr'])))
        self.pub_rl_wheel.publish(Float64(data=float(cmd_wheel['rl'])))
        self.pub_rr_wheel.publish(Float64(data=float(cmd_wheel['rr'])))

        # 5. COMPUTE ODOMETRY FROM FEEDBACK
        self.update_odometry()


    def update_odometry(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # sim time not ready yet

        if self.last_odom_time.nanoseconds == 0:
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds * 1e-9

        # Handle simulation reset or time jump
        if dt < 0.0:
            self.last_odom_time = now
            self.odom_x = 0.0
            self.odom_y = 0.0
            self.odom_yaw = 0.0
            return


        if dt == 0.0:
            return

        if dt > 1.0:
            dt = 0.02

        self.last_odom_time = now

        # Extract actual joint feedback
        wheel_joints = ['fl_wheel_joint', 'fr_wheel_joint', 'rl_wheel_joint', 'rr_wheel_joint']
        steer_joints = ['fl_steer_joint', 'fr_steer_joint', 'rl_steer_joint', 'rr_steer_joint']

        # If feedback is not ready yet, use commanded velocities as fallback
        v_components_x = []
        v_components_y = []
        rot_contributions = []

        modules = ['fl', 'fr', 'rl', 'rr']
        for mod in modules:
            s_joint = f"{mod}_steer_joint"
            w_joint = f"{mod}_wheel_joint"

            angle = self.current_joint_positions.get(s_joint, self.last_steer_angles[mod])
            w_vel = self.current_joint_velocities.get(w_joint, 0.0)
            lin_vel = w_vel * self.wheel_radius

            vx_i = lin_vel * math.cos(angle)
            vy_i = lin_vel * math.sin(angle)

            v_components_x.append(vx_i)
            v_components_y.append(vy_i)

            px, py = self.module_positions[mod]
            r_sq = px * px + py * py
            w_i = (px * vy_i - py * vx_i) / r_sq
            rot_contributions.append(w_i)

        # Average estimated body velocity
        v_body_x = float(np.mean(v_components_x))
        v_body_y = float(np.mean(v_components_y))
        w_body_z = float(np.mean(rot_contributions))

        # Integrate pose in world frame
        self.odom_yaw += w_body_z * dt
        self.odom_yaw = normalize_angle(self.odom_yaw)

        dx = (v_body_x * math.cos(self.odom_yaw) - v_body_y * math.sin(self.odom_yaw)) * dt
        dy = (v_body_x * math.sin(self.odom_yaw) + v_body_y * math.cos(self.odom_yaw)) * dt

        self.odom_x += dx
        self.odom_y += dy

        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, self.odom_yaw)

        # Broadcast TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_body_x
        odom.twist.twist.linear.y = v_body_y
        odom.twist.twist.angular.z = w_body_z
        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
