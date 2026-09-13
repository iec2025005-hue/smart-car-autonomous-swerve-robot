#!/usr/bin/env python3
"""
smart_car_ros/navigation_node.py

ROS 2 Navigation Adapter Node
- Directly wraps the original navigation/navigator.py (Navigator) without duplicating logic.
- The original Navigator class remains the single source of truth for all navigation
  decisions, behavioral modes (follow, avoid, patrol, idle), PID controller integration,
  and swerve commands.
"""

import os
import sys

# Add repository root to python sys.path to import existing navigation modules directly
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

import config
from navigation.navigator import Navigator
from vision.detector import Detection

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Float64MultiArray


class RosNavigator(Navigator):
    """Clean adapter extending original Navigator to publish ROS 2 /cmd_vel."""

    def __init__(self, publish_twist_fn, publish_pid_fn, base_speed_mps: float = 0.35):
        super().__init__()
        self._publish_twist = publish_twist_fn
        self._publish_pid = publish_pid_fn
        self._base_speed_mps = base_speed_mps

    def _cmd_swerve(self, vx: float, vy: float, omega: float) -> str:
        # 1. Execute original kinematics calculation & command string generation
        cmd_str = super()._cmd_swerve(vx, vy, omega)

        # 2. Scale to ROS REP-103 units (m/s and rad/s)
        scale = (self._base_speed_mps / float(config.BASE_SPEED)) if config.BASE_SPEED else 0.0023

        twist = Twist()
        twist.linear.x = float(vx * scale)
        twist.linear.y = float(-vy * scale)
        # Scale angular velocity (omega: 15 -> ~0.375 rad/s)
        twist.angular.z = float(-omega * 0.025)

        if self._mode != "idle":
            self._publish_twist(twist)

        # Publish PID diagnostics
        pid_diag = Float64MultiArray()
        last_err = getattr(self._pid, 'last_error', 0.0)
        last_p = getattr(self._pid, 'last_p', 0.0)
        last_i = getattr(self._pid, 'last_i', 0.0)
        last_d = getattr(self._pid, 'last_d', 0.0)
        last_out = getattr(self._pid, 'last_output', 0.0)
        pid_diag.data = [float(last_err), float(last_p), float(last_i), float(last_d), float(last_out), float(twist.linear.x), float(twist.linear.y)]
        self._publish_pid(pid_diag)

        return cmd_str


from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # ── Declare Parameters ─────────────────────────────────────
        self.declare_parameter('nav_mode', config.NAVIGATION_MODE)
        self.declare_parameter('base_speed', 0.35)           # 0.35 m/s

        nav_mode = self.get_parameter('nav_mode').get_parameter_value().string_value.lower()
        base_speed = self.get_parameter('base_speed').get_parameter_value().double_value

        # ── Publishers ─────────────────────────────────────────────
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_pid_diag = self.create_publisher(Float64MultiArray, '/pid/diagnostics', 10)
        self.pub_nav_state = self.create_publisher(String, '/navigation/state', 10)

        # Wrap existing Navigator directly
        self.navigator = RosNavigator(
            publish_twist_fn=self.pub_cmd_vel.publish,
            publish_pid_fn=self.pub_pid_diag.publish,
            base_speed_mps=base_speed
        )
        self.navigator.set_mode(nav_mode)

        self.last_target = None
        self.last_obstacle = None
        self.current_pose = (0.0, 0.0, 0.0) # (x, y, yaw)
        self.active_goal = None              # (gx, gy, gyaw)

        self.get_logger().info(f'Navigation Node ready [mode={nav_mode}, base_speed={base_speed:.2f} m/s]')

        # ── Subscribers ────────────────────────────────────────────
        self.sub_target = self.create_subscription(
            Point,
            '/target_point',
            self.target_callback,
            10
        )

        self.sub_obstacle = self.create_subscription(
            Point,
            '/obstacle_point',
            self.obstacle_callback,
            10
        )

        self.sub_set_mode = self.create_subscription(
            String,
            '/navigation/set_mode',
            self.set_mode_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # 50 Hz navigation control timer for waypoint following
        self.nav_timer = self.create_timer(0.02, self.waypoint_control_loop)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (p.x, p.y, yaw)

    def goal_pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        gyaw = math.atan2(siny_cosp, cosy_cosp)
        self.active_goal = (msg.pose.position.x, msg.pose.position.y, gyaw)
        self.navigator.set_mode("waypoint")
        self.navigator.state = "NAV_TO_GOAL"
        self.pub_nav_state.publish(String(data=self.navigator.state))
        self.get_logger().info(f"Received new goal pose: ({self.active_goal[0]:.2f}, {self.active_goal[1]:.2f}, yaw={math.degrees(gyaw):.1f}°)")

    def waypoint_control_loop(self):
        if self.navigator._mode != "waypoint" or self.active_goal is None:
            return

        gx, gy, gyaw = self.active_goal
        cx, cy, cyaw = self.current_pose

        ex = gx - cx
        ey = gy - cy
        eyaw = (gyaw - cyaw + math.pi) % (2.0 * math.pi) - math.pi
        dist = math.hypot(ex, ey)

        if dist < 0.04 and abs(eyaw) < 0.10:
            self.navigator.state = "GOAL_REACHED"
            self.pub_cmd_vel.publish(Twist())
            self.pub_nav_state.publish(String(data=self.navigator.state))
            self.active_goal = None
            self.get_logger().info(f"Target goal reached! Final pose: ({cx:.2f}, {cy:.2f}, yaw={math.degrees(cyaw):.1f}°)")
            return

        # Body frame errors
        vx_body = 2.0 * (ex * math.cos(cyaw) + ey * math.sin(cyaw))
        vy_body = 2.0 * (-ex * math.sin(cyaw) + ey * math.cos(cyaw))
        wz_body = 2.0 * eyaw

        # Distance-based deceleration
        sp = math.hypot(vx_body, vy_body)
        max_sp = 0.35
        if sp > max_sp:
            vx_body *= (max_sp / sp)
            vy_body *= (max_sp / sp)
        elif dist < 0.20:
            scale = max(0.40, dist / 0.20)
            vx_body *= scale
            vy_body *= scale

        wz_body = max(-0.8, min(0.8, wz_body))

        cmd = Twist()
        cmd.linear.x = float(vx_body)
        cmd.linear.y = float(vy_body)
        cmd.angular.z = float(wz_body)
        self.pub_cmd_vel.publish(cmd)
        self.navigator.state = "NAV_TO_GOAL"
        self.pub_nav_state.publish(String(data=self.navigator.state))

    def set_mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        self.navigator.set_mode(mode)
        if mode == "idle":
            self.pub_cmd_vel.publish(Twist())
        self.pub_nav_state.publish(String(data=self.navigator.state))
        self.get_logger().info(f"Navigation mode changed to: '{mode}'")

    def obstacle_callback(self, msg: Point):
        if msg.z > 0.0:  # Obstacle present
            w = int(msg.z)
            self.last_obstacle = Detection(
                label="obstacle",
                confidence=0.95,
                bbox=(int(msg.x - w // 2), int(msg.y - w // 2), w, w),
                center=(int(msg.x), int(msg.y))
            )
        else:
            self.last_obstacle = None

    def target_callback(self, msg: Point):
        detections = []
        if msg.z > 0.0:  # Target found
            w = int(msg.z) if msg.z > 1.0 else 40
            detections.append(
                Detection(
                    label="target",
                    confidence=1.0,
                    bbox=(int(msg.x - w // 2), int(msg.y - w // 2), w, w),
                    center=(int(msg.x), int(msg.y))
                )
            )
        if self.last_obstacle is not None:
            detections.append(self.last_obstacle)

        self.navigator.update(detections)
        self.pub_nav_state.publish(String(data=self.navigator.state))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, RuntimeError):
        pass
    except Exception:
        pass
    finally:
        if rclpy.ok():
            try:
                node.pub_cmd_vel.publish(Twist())
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()

