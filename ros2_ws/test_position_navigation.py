#!/usr/bin/env python3
"""
test_position_navigation.py -- Validates position-to-position holonomic waypoint navigation.
Tests multiple target poses starting from known origin.
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class PositionNavigationValidator(Node):
    def __init__(self):
        super().__init__('position_nav_validator')

        self.current_pose = None
        self.nav_state = None

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_state = self.create_subscription(String, '/navigation/state', self.state_cb, 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_set_mode = self.create_publisher(String, '/navigation/set_mode', 10)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (p.x, p.y, yaw)

    def state_cb(self, msg: String):
        self.nav_state = msg.data.strip()

    def send_goal_and_wait(self, gx: float, gy: float, gyaw_deg: float):
        gyaw_rad = math.radians(gyaw_deg)
        print(f"\n────────────────────────────────────────────────────────────────────────")
        print(f"▶ COMMANDING WAYPOINT GOAL: ({gx:+.2f} m, {gy:+.2f} m, heading={gyaw_deg:+.1f}°)")
        print(f"────────────────────────────────────────────────────────────────────────")

        # Wait for odom
        t0 = time.time()
        while self.current_pose is None and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        x_start, y_start, yaw_start = self.current_pose
        travel_dist = math.hypot(gx - x_start, gy - y_start)
        timeout_sec = max(15.0, travel_dist * 8.0 + 8.0)
        print(f"  Start Pose : x={x_start:+.3f} m, y={y_start:+.3f} m, yaw={math.degrees(yaw_start):+.1f}° (Dist={travel_dist:.2f} m, Timeout={timeout_sec:.1f} s)")

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.position.z = 0.0

        # Orientation quaternion
        goal.pose.orientation.z = math.sin(gyaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(gyaw_rad / 2.0)

        self.pub_goal.publish(goal)

        t_start = time.time()
        prev_dist = 999.0
        slowed_down = False

        while time.time() - t_start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_pose is not None:
                cx, cy, cyaw = self.current_pose
                dist = math.hypot(gx - cx, gy - cy)
                eyaw = math.degrees((gyaw_rad - cyaw + math.pi) % (2.0 * math.pi) - math.pi)

                # Check deceleration near target
                if dist < 0.20 and prev_dist >= 0.20:
                    slowed_down = True
                    print(f"  [DECELERATION] Robot entered deceleration zone (dist={dist:.3f} m < 0.20 m)")

                prev_dist = dist

                if dist < 0.05 and abs(eyaw) < 6.0:
                    print(f"  [ARRIVED] Target reached within tolerance in {time.time() - t_start:.2f} s!")
                    break

            time.sleep(0.05)

        x_end, y_end, yaw_end = self.current_pose
        final_dist_err = math.hypot(gx - x_end, gy - y_end)
        final_yaw_err = math.degrees((gyaw_rad - yaw_end + math.pi) % (2.0 * math.pi) - math.pi)

        passed = (final_dist_err <= 0.06) and (abs(final_yaw_err) <= 8.0)

        print(f"  End Pose   : x={x_end:+.3f} m, y={y_end:+.3f} m, yaw={math.degrees(yaw_end):+.1f}°")
        print(f"  Error      : Pos Error={final_dist_err*1000:.1f} mm, Yaw Error={final_yaw_err:+.1f}°")
        print(f"  Deceleration Verified: {'✅ YES' if slowed_down else '⚠️ Fast approach'}")
        print(f"  Result     : {'✅ PASS' if passed else '❌ FAIL'}")

        return {
            'goal': (gx, gy, gyaw_deg),
            'start': (x_start, y_start, math.degrees(yaw_start)),
            'end': (x_end, y_end, math.degrees(yaw_end)),
            'pos_error_mm': final_dist_err * 1000.0,
            'yaw_error_deg': final_yaw_err,
            'slowed_down': slowed_down,
            'pass': passed
        }


def main():
    rclpy.init()
    node = PositionNavigationValidator()

    print("═══════════════════════════════════════════════════════════════════════════")
    print("  SMART CAR POSITION-TO-POSITION HOLONOMIC NAVIGATION BENCHMARK")
    print("═══════════════════════════════════════════════════════════════════════════")

    test_waypoints = [
        ("WAYPOINT 1: Goal (1.0, 0.0, 0.0°)", +1.00, +0.00, +0.0),
        ("WAYPOINT 2: Goal (0.0, 1.0, 0.0°)", +0.00, +1.00, +0.0),
        ("WAYPOINT 3: Goal (1.0, 1.0, 45.0°)", +1.00, +1.00, +45.0),
        ("WAYPOINT 4: Goal (-1.0, 1.0, 90.0°)", -1.00, +1.00, +90.0),
        ("WAYPOINT 5: Goal (-1.0, -1.0, -135.0°)", -1.00, -1.00, -135.0),
        ("WAYPOINT 6: Goal (0.5, 1.5, 30.0°)", +0.50, +1.50, +30.0),
        ("WAYPOINT 7: Precision Return to Origin (0.0, 0.0, 0.0°)", +0.00, +0.00, +0.0),
    ]

    # Ensure DDS discovery and subscription connection
    t_wait = time.time()
    while (node.pub_goal.get_subscription_count() < 1 or node.current_pose is None) and time.time() - t_wait < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    time.sleep(0.5)

    results = []
    for name, gx, gy, gyaw in test_waypoints:
        print(f"\n>>> Executing {name}")
        res = node.send_goal_and_wait(gx, gy, gyaw)
        results.append((name, res))
        time.sleep(0.5)

    # Summary
    print("\n═══════════════════════════════════════════════════════════════════════════")
    print("  POSITION NAVIGATION BENCHMARK SCORECARD")
    print("═══════════════════════════════════════════════════════════════════════════")
    all_passed = True
    for name, res in results:
        status_str = "✅ PASS" if res['pass'] else "❌ FAIL"
        if not res['pass']:
            all_passed = False
        print(f"  {name:48} | Pos Err: {res['pos_error_mm']:5.1f} mm | Yaw Err: {res['yaw_error_deg']:+5.1f}° | {status_str}")

    print(f"\nOVERALL STATUS: {'✅ ALL TESTS PASSED' if all_passed else '❌ SOME TESTS FAILED'}")
    print("═══════════════════════════════════════════════════════════════════════════")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
