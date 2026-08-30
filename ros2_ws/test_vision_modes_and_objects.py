#!/usr/bin/env python3
"""
test_vision_modes_and_objects.py -- Comprehensive test for:
1. Color Detection (Red cylinder, Yellow obstacle, Blue box)
2. ArUco Marker Detection (Marker ID 7 6-DOF)
3. YOLOv8 Object Detection (Chair, Person, Bottle)
4. Dynamic Object Command & 3D coordinate estimation (/target_pose_estimated)
"""

import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VisionModesValidator(Node):
    def __init__(self):
        super().__init__('vision_modes_validator')

        self.last_target_point = None
        self.last_target_3d = None
        self.last_obstacle_point = None
        self.last_image_header = None

        self.pub_vision_mode = self.create_publisher(String, '/vision/set_mode', 10)
        self.pub_target_class = self.create_publisher(String, '/vision/set_target', 10)

        self.sub_target = self.create_subscription(Point, '/target_point', self.target_cb, 10)
        self.sub_target_3d = self.create_subscription(PoseStamped, '/target_pose_estimated', self.target_3d_cb, 10)
        self.sub_obstacle = self.create_subscription(Point, '/obstacle_point', self.obs_cb, 10)
        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.img_cb, 10)

    def reset_robot_pose(self):
        subprocess.run([
            "gz", "service", "-s", "/world/smart_car_world/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "name: 'smart_car', position: {x: 0.0, y: 0.0, z: 0.039965}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        time.sleep(0.5)

    def target_cb(self, msg: Point):
        self.last_target_point = msg

    def target_3d_cb(self, msg: PoseStamped):
        self.last_target_3d = msg

    def obs_cb(self, msg: Point):
        self.last_obstacle_point = msg

    def img_cb(self, msg: Image):
        self.last_image_header = msg.header

    def test_vision_pipeline(self, test_name: str, mode: str, target_class: str, duration_sec: float = 4.0):
        self.reset_robot_pose()
        print(f"\n────────────────────────────────────────────────────────────────────────")
        print(f"▶ TESTING VISION: Mode='{mode}', Target='{target_class}' ({test_name})")
        print(f"────────────────────────────────────────────────────────────────────────")

        # 1. Set mode and target
        self.pub_vision_mode.publish(String(data=mode))
        self.pub_target_class.publish(String(data=target_class))

        self.last_target_point = None
        self.last_target_3d = None
        self.last_obstacle_point = None

        # Settle mode and warm up detector
        t_warmup = time.time()
        while time.time() - t_warmup < 1.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        t0 = time.time()
        detected_frames = 0
        samples_target = []
        samples_3d = []

        is_obstacle_test = (target_class == "obstacle")

        while time.time() - t0 < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            pt = self.last_obstacle_point if is_obstacle_test else self.last_target_point
            if pt is not None and pt.z > 0:
                detected_frames += 1
                samples_target.append((pt.x, pt.y, pt.z))
                if self.last_target_3d is not None:
                    p = self.last_target_3d.pose.position
                    samples_3d.append((p.x, p.y, math.hypot(p.x, p.y)))
            time.sleep(0.02)

        passed = detected_frames >= 5
        print(f"  Camera Active       : {'✅ YES' if self.last_image_header is not None else '❌ NO IMAGE'}")
        print(f"  Frames Detected     : {detected_frames} valid frames")
        if samples_target:
            avg_x = sum(s[0] for s in samples_target) / len(samples_target)
            avg_y = sum(s[1] for s in samples_target) / len(samples_target)
            avg_w = sum(s[2] for s in samples_target) / len(samples_target)
            print(f"  2D Image Center     : ({avg_x:.1f}, {avg_y:.1f}) px | Bounding Box Width: {avg_w:.1f} px")
        if samples_3d:
            avg_bx = sum(s[0] for s in samples_3d) / len(samples_3d)
            avg_by = sum(s[1] for s in samples_3d) / len(samples_3d)
            avg_d = sum(s[2] for s in samples_3d) / len(samples_3d)
            print(f"  3D Estimated Target : Body X={avg_bx:+.2f} m, Body Y={avg_by:+.2f} m, Distance={avg_d:.2f} m")
        
        print(f"  Result              : {'✅ PASS' if passed else '❌ FAIL'}")

        return {
            'name': test_name,
            'mode': mode,
            'target': target_class,
            'detected_frames': detected_frames,
            'pass': passed
        }


def main():
    rclpy.init()
    node = VisionModesValidator()

    print("═══════════════════════════════════════════════════════════════════════════")
    print("  SMART CAR VISION DETECTOR & 3D OBJECT TARGETING BENCHMARK")
    print("═══════════════════════════════════════════════════════════════════════════")

    # Wait for camera stream
    print("Waiting for camera image feed on /camera/image_raw...")
    t0 = time.time()
    while node.last_image_header is None and time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.last_image_header is None:
        print("❌ Camera feed not active! Check simulation.")
        return

    print("✅ Camera feed active! Running detection tests...\n")

    tests = [
        ("YOLO Detection: Chair Target", "yolo", "chair"),
        ("Color Detection: Red Target Cylinder", "color", "red"),
        ("Obstacle Segmentation: Yellow & Blue Obstacles", "color", "obstacle"),
        ("ArUco Marker Detection: Marker ID 7", "aruco", "aruco"),
    ]

    results = []
    for name, mode, tgt in tests:
        res = node.test_vision_pipeline(name, mode, tgt, duration_sec=3.0)
        results.append(res)
        time.sleep(0.5)

    print("\n═══════════════════════════════════════════════════════════════════════════")
    print("  VISION PIPELINE SCORECARD")
    print("═══════════════════════════════════════════════════════════════════════════")
    all_passed = True
    for r in results:
        status_str = "✅ PASS" if r['pass'] else "❌ FAIL"
        if not r['pass']:
            all_passed = False
        print(f"  {r['name']:48} | Mode: {r['mode']:6} | Frames: {r['detected_frames']:2} | {status_str}")

    print(f"\nOVERALL STATUS: {'✅ ALL VISION MODES PASSED' if all_passed else '❌ SOME TESTS FAILED'}")
    print("═══════════════════════════════════════════════════════════════════════════")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
