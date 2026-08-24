#!/usr/bin/env python3
"""
smart_car_ros/navigation_node.py

ROS 2 High-Level Navigation Node
- Consumes /target_point (geometry_msgs/Point) from vision_node.
- Executes the existing PID control (navigation/pid.py) and behavioral strategies (Follow, Avoid, Patrol, Idle).
- Publishes /cmd_vel (geometry_msgs/Twist) to drive the Swerve Controller in simulation or physical hardware.
- Supports dynamic mode switching via /navigation/set_mode (std_msgs/String).
"""

import os
import sys
import math

# Add repository root to python sys.path to import existing navigation modules directly
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

import config
from navigation.pid import PIDController

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # ── Declare Parameters ─────────────────────────────────────
        self.declare_parameter('nav_mode', 'follow')
        self.declare_parameter('base_speed', 0.4)           # 0.4 m/s
        self.declare_parameter('frame_center_x', 320)       # 640x480 center
        self.declare_parameter('tolerance_px', 40)          # 40 px deadband

        self.mode = self.get_parameter('nav_mode').get_parameter_value().string_value.lower()
        self.base_speed = self.get_parameter('base_speed').get_parameter_value().double_value
        self.frame_cx = self.get_parameter('frame_center_x').get_parameter_value().integer_value
        self.tolerance = self.get_parameter('tolerance_px').get_parameter_value().integer_value

        # Existing PID controller instance
        self.pid = PIDController(
            kp=0.003,
            ki=0.0001,
            kd=0.0005,
            output_limits=(-0.6, 0.6)
        )

        self.get_logger().info(f'Navigation Node ready [mode={self.mode}, base_speed={self.base_speed:.2f} m/s]')

        # ── Publishers ─────────────────────────────────────────────
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Subscribers ────────────────────────────────────────────
        self.sub_target = self.create_subscription(
            Point,
            '/target_point',
            self.target_callback,
            10
        )

        self.sub_set_mode = self.create_subscription(
            String,
            '/navigation/set_mode',
            self.set_mode_callback,
            10
        )

    def set_mode_callback(self, msg: String):
        old_mode = self.mode
        self.mode = msg.data.strip().lower()
        self.pid.reset()
        self.get_logger().info(f"Navigation mode changed: '{old_mode}' -> '{self.mode}'")

    def target_callback(self, msg: Point):
        twist = Twist()

        # 1. IDLE MODE
        if self.mode == 'idle':
            self.pub_cmd_vel.publish(twist)
            return

        # Target lost flag: z < 0
        target_found = (msg.z > 0.0)

        # 2. FOLLOW MODE (Move forward + strafe laterally using PID)
        if self.mode == 'follow':
            if not target_found:
                # No target detected -> stop
                self.pub_cmd_vel.publish(twist)
                return

            error_x = msg.x - self.frame_cx
            # In ROS REP-103, +Y is Left, -Y is Right.
            # If target is to the right (error_x > 0), strafe right (-Y).
            # If target is to the left (error_x < 0), strafe left (+Y).
            if abs(error_x) > self.tolerance:
                correction = -self.pid.compute(error_x)
                twist.linear.y = float(correction)
            else:
                twist.linear.y = 0.0

            twist.linear.x = float(self.base_speed)
            twist.angular.z = 0.0

        # 3. AVOID MODE (Strafe away from obstacle)
        elif self.mode == 'avoid':
            if not target_found:
                # Path clear -> drive forward
                twist.linear.x = float(self.base_speed)
            else:
                error_x = msg.x - self.frame_cx
                twist.linear.x = float(self.base_speed * 0.5)
                if error_x < 0:
                    # Obstacle on Left -> Strafe Right (-Y)
                    twist.linear.y = -float(self.base_speed)
                else:
                    # Obstacle on Right or Center -> Strafe Left (+Y)
                    twist.linear.y = float(self.base_speed)

        # 4. PATROL MODE (Drive straight, stop if obstacle in front)
        elif self.mode == 'patrol':
            if target_found and (abs(msg.x - self.frame_cx) < self.tolerance * 2):
                # Obstacle directly ahead -> Stop
                twist.linear.x = 0.0
                twist.linear.y = 0.0
            else:
                twist.linear.x = float(self.base_speed)
                twist.linear.y = 0.0

        else:
            self.get_logger().warn(f'Unknown nav_mode: {self.mode}')
            twist = Twist()

        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
