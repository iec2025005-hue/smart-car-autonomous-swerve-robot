#!/usr/bin/env python3
"""
smart_car_ros/vision_node.py

ROS 2 Vision Processing Node
- Subscribes to /camera/image_raw (sensor_msgs/Image) from Gazebo or hardware camera driver.
- Executes the existing modular detectors from vision/ (Color, ArUco, Lane, YOLO).
- Uses the existing OpenCV Tracker (vision/tracker.py) for tracking between inference.
- Annotates the frame using existing HUD Overlay (display/overlay.py).
- Publishes /target_point (geometry_msgs/Point) and /camera/image_annotated (sensor_msgs/Image).
"""

import os
import sys
import time
import cv2
import numpy as np

# Add repository root to python sys.path to import existing vision modules directly
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

import config
from vision.detector import create_detector, Detection
from vision.tracker import Tracker
from display.overlay import Overlay
from utils.fps import FPSCounter

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ── Declare Parameters ─────────────────────────────────────
        self.declare_parameter('vision_mode', config.VISION_MODE)
        self.declare_parameter('target_class', 'chair')
        self.declare_parameter('show_preview', False)

        self.vision_mode = self.get_parameter('vision_mode').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.show_preview = self.get_parameter('show_preview').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.fps_counter = FPSCounter()
        self.overlay = Overlay()
        self.tracker = Tracker()

        # Telemetry cache for HUD
        self.current_nav_state = "IDLE"
        self.current_pid_err = 0.0
        self.current_cmd_vel = (0.0, 0.0, 0.0)

        # Initialize detector using existing factory
        self.get_logger().info(f'Initializing vision detector for mode: {self.vision_mode}...')
        try:
            self.detector = create_detector(self.vision_mode)
            if hasattr(self.detector, 'set_target'):
                self.detector.set_target(self.target_class)
        except Exception as e:
            self.get_logger().warn(f'Could not initialize mode {self.vision_mode} ({e}). Falling back to "color" mode.')
            self.vision_mode = 'color'
            self.detector = create_detector('color')

        self.get_logger().info(f'Vision Node ready [mode={self.vision_mode}, target={self.target_class}]')

        # ── Publishers ─────────────────────────────────────────────
        self.pub_target = self.create_publisher(Point, '/target_point', 10)
        self.pub_target_estimated = self.create_publisher(PoseStamped, '/target_pose_estimated', 10)
        self.pub_obstacle = self.create_publisher(Point, '/obstacle_point', 10)
        self.pub_annotated = self.create_publisher(Image, '/camera/image_annotated', 10)

        # ── Subscribers ────────────────────────────────────────────
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.sub_mode = self.create_subscription(
            String,
            '/vision/set_mode',
            self.set_mode_callback,
            10
        )

        self.sub_target = self.create_subscription(
            String,
            '/vision/set_target',
            self.set_target_callback,
            10
        )

        self.sub_state = self.create_subscription(
            String,
            '/navigation/state',
            self.nav_state_callback,
            10
        )

        self.sub_pid = self.create_subscription(
            Float64MultiArray,
            '/pid/diagnostics',
            self.pid_callback,
            10
        )

        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def set_mode_callback(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode in ["yolo", "color", "aruco", "lane"]:
            try:
                new_detector = create_detector(new_mode)
                if hasattr(new_detector, 'set_target'):
                    new_detector.set_target(self.target_class)
                if self.detector is not None:
                    self.detector.release()
                self.detector = new_detector
                self.vision_mode = new_mode
                self.tracker.reset()
                self.get_logger().info(f'Vision mode switched to: {new_mode}')
            except Exception as e:
                self.get_logger().error(f'Failed to switch vision mode to {new_mode}: {e}')

    def set_target_callback(self, msg: String):
        target_name = msg.data.strip().lower()
        self.target_class = target_name

        # Auto-switch mode based on target name
        target_mode = "yolo"
        if target_name in ["red", "color", "green", "blue_target"]:
            target_mode = "color"
        elif target_name in ["aruco", "marker", "7", "aruco7", "aruco_7"]:
            target_mode = "aruco"
        elif target_name in ["lane", "lanes", "road"]:
            target_mode = "lane"

        if target_mode != self.vision_mode:
            try:
                new_detector = create_detector(target_mode)
                if self.detector is not None:
                    self.detector.release()
                self.detector = new_detector
                self.vision_mode = target_mode
                self.get_logger().info(f"Auto-switched vision mode to '{target_mode}' for target '{target_name}'")
            except Exception as e:
                self.get_logger().error(f"Failed to switch mode to {target_mode}: {e}")

        if hasattr(self.detector, 'set_target'):
            self.detector.set_target(target_name)
            self.tracker.reset()
            self.get_logger().info(f"Target object set to: '{target_name}'")

    def nav_state_callback(self, msg: String):
        self.current_nav_state = msg.data.strip()

    def pid_callback(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= 1:
            self.current_pid_err = float(msg.data[0])

    def cmd_vel_callback(self, msg: Twist):
        self.current_cmd_vel = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    def image_callback(self, msg: Image):
        # 1. Convert ROS Image message to OpenCV BGR image
        try:
            if msg.encoding in ('rgb8', 'bgr8'):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding == 'mono8':
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge conversion error: {e}')
            return

        self.fps_counter.tick()
        fps_val = self.fps_counter.get()

        # 2. Run active detector
        detections = []
        if self.detector is not None:
            try:
                detections = self.detector.detect(frame)
            except Exception as e:
                self.get_logger().debug(f'Detector error: {e}')
                detections = []

        targets = [d for d in detections if d.label != "obstacle"]

        # 3. Arena Obstacle Detection (Yellow Cylinder & Blue Box)
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv, np.array([15, 80, 70]), np.array([40, 255, 255]))
            mask_blue = cv2.inRange(hsv, np.array([100, 80, 70]), np.array([130, 255, 255]))
            mask_obs = cv2.bitwise_or(mask_yellow, mask_blue)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask_obs = cv2.morphologyEx(mask_obs, cv2.MORPH_OPEN, kernel, iterations=2)
            obs_contours, _ = cv2.findContours(mask_obs, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in obs_contours:
                area = cv2.contourArea(cnt)
                if area > 100:
                    x, y, w, h = cv2.boundingRect(cnt)
                    # Ground-level obstacle in camera view
                    if y + h > frame.shape[0] * 0.20:
                        cx, cy = x + w // 2, y + h // 2
                        detections.append(
                            Detection(
                                label="obstacle",
                                confidence=0.95,
                                bbox=(x, y, w, h),
                                center=(cx, cy),
                                extra={"area": area, "color": "obstacle"}
                            )
                        )
        except Exception as e:
            self.get_logger().debug(f'Obstacle segmentation error: {e}')


        # 4. Object Tracking between frames
        targets = [d for d in detections if d.label != "obstacle"]
        obstacles = [d for d in detections if d.label == "obstacle"]

        if targets:
            primary_det = max(targets, key=lambda d: d.confidence)
            if not self.tracker.is_tracking:
                self.tracker.init(frame, primary_det.bbox)
        else:
            ok, _, _ = self.tracker.update(frame)
            if ok:
                det = self.tracker.to_detection()
                if det:
                    targets = [det]
                    detections.append(det)

        # 5. Compute 3D target coordinate estimation and target/obstacle point messages
        target_msg = Point()
        target_label = "None"
        target_conf = 0.0
        target_3d_str = "Target 3D: None"

        nominal_widths = {
            'chair': 0.65,
            'person': 0.60,
            'bottle': 0.35,
            'color_target': 0.24,
            'red': 0.24,
            'aruco': 0.35,
        }

        if targets:
            primary_target = max(targets, key=lambda d: d.confidence)
            target_msg.x = float(primary_target.center[0])
            target_msg.y = float(primary_target.center[1])
            target_msg.z = float(primary_target.bbox[2])  # Width of target bbox
            target_label = primary_target.label
            target_conf = primary_target.confidence * 100.0

            # 3D Position Estimation using Pinhole Camera Geometry
            # fx = (640 / 2) / tan(62.2° / 2) ≈ 530.4 px
            fx = 530.4
            cx_img, cy_img = primary_target.center
            w_px = max(1.0, float(primary_target.bbox[2]))
            
            # Lookup nominal physical object width
            obj_key = self.target_class.lower()
            w_real = nominal_widths.get(obj_key, 0.50)
            
            d_est = (w_real * fx) / w_px
            bearing_rad = math.atan2(cx_img - 320.0, fx)
            
            # Robot body frame (+X forward, +Y left)
            x_body = d_est * math.cos(bearing_rad)
            y_body = -d_est * math.sin(bearing_rad)
            target_3d_str = f"Target 3D: X={x_body:.2f}m, Y={y_body:+.2f}m, D={d_est:.2f}m"

            # Publish estimated 3D pose
            est_pose = PoseStamped()
            est_pose.header = msg.header
            est_pose.header.frame_id = 'base_link'
            est_pose.pose.position.x = float(x_body)
            est_pose.pose.position.y = float(y_body)
            est_pose.pose.position.z = 0.0
            self.pub_target_estimated.publish(est_pose)
        else:
            target_msg.x = 0.0
            target_msg.y = 0.0
            target_msg.z = -1.0  # Lost

        self.pub_target.publish(target_msg)

        obs_msg = Point()
        if obstacles:
            primary_obs = max(obstacles, key=lambda d: d.bbox[2] * d.bbox[3])
            obs_msg.x = float(primary_obs.center[0])
            obs_msg.y = float(primary_obs.center[1])
            obs_msg.z = float(primary_obs.bbox[2])  # Width of obstacle bbox
        else:
            obs_msg.x = 0.0
            obs_msg.y = 0.0
            obs_msg.z = -1.0  # None

        self.pub_obstacle.publish(obs_msg)

        # 6. Render Comprehensive HUD Overlay
        annotated_frame = frame.copy()
        h, w_img, _ = annotated_frame.shape

        # Draw base detection boxes
        self.overlay.draw(
            annotated_frame,
            detections,
            fps=fps_val,
            vision_mode=self.vision_mode,
            nav_command=""
        )

        # Draw Custom Comprehensive Smart Car HUD telemetry banner on top
        hud_bg = annotated_frame.copy()
        cv2.rectangle(hud_bg, (5, 5), (420, 180), (20, 20, 20), -1)
        cv2.addWeighted(hud_bg, 0.75, annotated_frame, 0.25, 0, annotated_frame)
        cv2.rectangle(annotated_frame, (5, 5), (420, 180), (80, 80, 80), 1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.40
        c_white = (255, 255, 255)
        c_green = (0, 255, 0)
        c_cyan = (255, 255, 0)
        c_yellow = (0, 255, 255)
        c_red = (0, 0, 255)

        # HUD Lines
        cv2.putText(annotated_frame, f"Target: {self.target_class}", (12, 20), font, font_scale, c_cyan, 1)
        det_str = f"Detection: {target_label}" if target_msg.z > 0 else "Detection: Searching..."
        cv2.putText(annotated_frame, det_str, (12, 36), font, font_scale, c_green if target_msg.z > 0 else c_yellow, 1)
        conf_str = f"Confidence: {target_conf:.1f}%" if target_msg.z > 0 else "Confidence: 0.0%"
        cv2.putText(annotated_frame, conf_str, (12, 52), font, font_scale, c_white, 1)
        tgt_center_str = f"Target center: {target_msg.x:.0f},{target_msg.y:.0f}" if target_msg.z > 0 else "Target center: None"
        cv2.putText(annotated_frame, tgt_center_str, (12, 68), font, font_scale, c_white, 1)
        cv2.putText(annotated_frame, target_3d_str, (12, 84), font, font_scale, c_cyan if target_msg.z > 0 else c_white, 1)
        cv2.putText(annotated_frame, f"Navigation state: {self.current_nav_state}", (12, 100), font, font_scale, c_yellow, 1)
        cv2.putText(annotated_frame, f"PID error: {self.current_pid_err:+.1f} px", (12, 116), font, font_scale, c_white, 1)
        cv2.putText(annotated_frame, f"Command velocity: vx={self.current_cmd_vel[0]:+.2f}, vy={self.current_cmd_vel[1]:+.2f}, wz={self.current_cmd_vel[2]:+.2f}", (12, 132), font, font_scale, c_white, 1)
        obs_str = f"Obstacle: YES ({obs_msg.x:.0f},{obs_msg.y:.0f})" if obs_msg.z > 0 else "Obstacle: NO"
        cv2.putText(annotated_frame, obs_str, (12, 148), font, font_scale, c_red if obs_msg.z > 0 else c_green, 1)
        cv2.putText(annotated_frame, f"FPS: {fps_val:.1f}", (12, 164), font, font_scale, (180, 180, 180), 1)

        # 7. Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            annotated_msg.header = msg.header
            self.pub_annotated.publish(annotated_msg)
        except Exception as e:
            self.get_logger().debug(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, RuntimeError):
        pass
    except Exception:
        pass
    finally:
        try:
            if hasattr(node, 'detector') and node.detector is not None:
                node.detector.release()
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
