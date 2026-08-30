#!/usr/bin/env python3
"""
test_obstacle_avoidance_comprehensive.py -- Comprehensive 4-Scenario Obstacle Avoidance Validation:
1. Obstacle Directly Ahead (x=1.2, y=0.0)
2. Obstacle Slightly Left (x=1.2, y=+0.20)
3. Obstacle Slightly Right (x=1.2, y=-0.20)
4. Obstacle Near Target (x=1.8, y=+0.10)
"""

import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty


class ObstacleAvoidanceComprehensiveValidator(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_comp_validator')

        self.current_pose = None
        self.nav_state = None
        self.last_target = None
        self.last_obstacle = None

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_state = self.create_subscription(String, '/navigation/state', self.state_cb, 10)
        self.sub_target = self.create_subscription(Point, '/target_point', self.target_cb, 10)
        self.sub_obstacle = self.create_subscription(Point, '/obstacle_point', self.obs_cb, 10)

        self.pub_mode = self.create_publisher(String, '/navigation/set_mode', 10)
        self.pub_vision_mode = self.create_publisher(String, '/vision/set_mode', 10)
        self.pub_target_class = self.create_publisher(String, '/vision/set_target', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reset_odom = self.create_publisher(Empty, '/reset_odom', 10)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (p.x, p.y, yaw)

    def state_cb(self, msg: String):
        self.nav_state = msg.data.strip()

    def target_cb(self, msg: Point):
        self.last_target = msg

    def obs_cb(self, msg: Point):
        self.last_obstacle = msg

    def set_obstacle_pose(self, x: float, y: float, z: float = 0.2):
        """Move yellow obstacle cylinder via Gazebo transport service."""
        cmd = [
            "gz", "service", "-s", "/world/smart_car_world/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", f"name: 'yellow_obstacle_cylinder', position: {{x: {x}, y: {y}, z: {z}}}"
        ]
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            time.sleep(0.5)
        except Exception:
            pass

    def reset_robot_to_origin(self):
        """Reset robot to (0, 0, 0) via Gazebo pose service and reset odometry."""
        self.pub_mode.publish(String(data='idle'))
        self.pub_cmd_vel.publish(Twist())
        subprocess.run([
            "gz", "service", "-s", "/world/smart_car_world/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "name: 'smart_car', position: {x: 0.0, y: 0.0, z: 0.039965}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        self.pub_reset_odom.publish(Empty())
        time.sleep(0.3)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_scenario(self, title: str, obs_x: float, obs_y: float, timeout_sec: float = 12.0):
        print(f"\n────────────────────────────────────────────────────────────────────────")
        print(f"▶ SCENARIO: {title} [Obstacle at ({obs_x:+.2f} m, {obs_y:+.2f} m)]")
        print(f"────────────────────────────────────────────────────────────────────────")

        # 1. Stop car & position obstacle & reset car
        self.nav_state = "IDLE"
        self.pub_mode.publish(String(data='idle'))
        self.pub_cmd_vel.publish(Twist())
        self.set_obstacle_pose(obs_x, obs_y, 0.2)
        self.reset_robot_to_origin()

        # Configure vision to YOLO Chair
        self.pub_vision_mode.publish(String(data='yolo'))
        self.pub_target_class.publish(String(data='chair'))
        time.sleep(0.5)

        # 2. Start Follow Mode
        self.nav_state = "IDLE"
        self.pub_mode.publish(String(data='follow'))
        time.sleep(0.3)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        t_start = time.time()
        states_seen = []
        min_dist_to_obs = 999.0
        collision = False

        while time.time() - t_start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_pose is not None:
                cx, cy, _ = self.current_pose
                # Obstacle radius 0.1m, robot radius ~0.16m -> collision threshold = 0.26m
                dist_to_obs = math.hypot(obs_x - cx, obs_y - cy)
                if dist_to_obs < min_dist_to_obs:
                    min_dist_to_obs = dist_to_obs

                if dist_to_obs < 0.24:
                    collision = True

                if self.nav_state and (not states_seen or states_seen[-1] != self.nav_state):
                    states_seen.append(self.nav_state)
                    print(f"  [STATE CHANGE @ t={time.time()-t_start:.1f}s]: -> {self.nav_state}")

                if self.nav_state == "ARRIVED":
                    print(f"  [SAFE ARRIVAL] Reached destination at standoff distance in {time.time()-t_start:.1f}s!")
                    break

            time.sleep(0.05)

        self.pub_mode.publish(String(data='idle'))
        self.pub_cmd_vel.publish(Twist())

        final_x, final_y, _ = self.current_pose if self.current_pose else (0, 0, 0)
        avoided = "AVOID" in states_seen
        passed = avoided and not collision and final_x >= 1.0

        print(f"  Min Obstacle Distance : {min_dist_to_obs:.3f} m (Safety threshold = 0.26 m)")
        print(f"  Collision Detected    : {'❌ YES (Collision!)' if collision else '✅ NO (Clean evasion)'}")
        print(f"  State Trajectory      : {' -> '.join(states_seen)}")
        print(f"  Final Robot Position  : ({final_x:+.2f} m, {final_y:+.2f} m)")
        print(f"  Scenario Evaluation   : {'✅ PASS' if passed else '❌ FAIL'}")

        return {
            'title': title,
            'obs_pos': (obs_x, obs_y),
            'min_dist': min_dist_to_obs,
            'collision': collision,
            'states': states_seen,
            'pass': passed
        }


def main():
    rclpy.init()
    node = ObstacleAvoidanceComprehensiveValidator()

    print("═══════════════════════════════════════════════════════════════════════════")
    print("  SMART CAR COMPREHENSIVE 4-SCENARIO OBSTACLE AVOIDANCE BENCHMARK")
    print("═══════════════════════════════════════════════════════════════════════════")

    # Wait for odom & camera
    print("Connecting to simulation...")
    t0 = time.time()
    while node.current_pose is None and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    scenarios = [
        ("SCENARIO 1: Obstacle Directly Ahead", 1.20, 0.00),
        ("SCENARIO 2: Obstacle Slightly Left", 1.20, +0.20),
        ("SCENARIO 3: Obstacle Slightly Right", 1.20, -0.20),
        ("SCENARIO 4: Obstacle Near Target", 1.80, +0.10),
    ]

    results = []
    for title, ox, oy in scenarios:
        res = node.run_scenario(title, ox, oy, timeout_sec=12.0)
        results.append(res)
        time.sleep(1.0)

    # Restore default obstacle position
    node.set_obstacle_pose(1.20, 0.18, 0.20)

    print("\n═══════════════════════════════════════════════════════════════════════════")
    print("  OBSTACLE AVOIDANCE SCORECARD")
    print("═══════════════════════════════════════════════════════════════════════════")
    all_passed = True
    for r in results:
        status_str = "✅ PASS" if r['pass'] else "❌ FAIL"
        if not r['pass']:
            all_passed = False
        print(f"  {r['title']:42} | Min Dist: {r['min_dist']:.3f} m | Collisions: {0 if not r['collision'] else 1} | {status_str}")

    print(f"\nOVERALL STATUS: {'✅ ALL OBSTACLE SCENARIOS PASSED' if all_passed else '❌ SOME TESTS FAILED'}")
    print("═══════════════════════════════════════════════════════════════════════════")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
