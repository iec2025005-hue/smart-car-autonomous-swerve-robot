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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ── Declare Parameters ─────────────────────────────────────
        self.declare_parameter('vision_mode', config.VISION_MODE)
        self.declare_parameter('target_class', 'all')
        self.declare_parameter('show_preview', False)

        self.vision_mode = self.get_parameter('vision_mode').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.show_preview = self.get_parameter('show_preview').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.fps_counter = FPSCounter()
        self.overlay = Overlay()
        self.tracker = Tracker()

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

        self.get_logger().info(f'Vision Node ready [mode={self.vision_mode}]')

        # ── Publishers ─────────────────────────────────────────────
        self.pub_target = self.create_publisher(Point, '/target_point', 10)
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

    def set_mode_callback(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode in ["yolo", "color", "aruco", "lane"]:
            try:
                new_detector = create_detector(new_mode)
                if self.detector is not None:
                    self.detector.release()
                self.detector = new_detector
                self.vision_mode = new_mode
                self.tracker.reset()
                self.get_logger().info(f'Vision mode switched to: {new_mode}')
            except Exception as e:
                self.get_logger().error(f'Failed to switch vision mode to {new_mode}: {e}')

    def image_callback(self, msg: Image):
        # 1. Convert ROS Image message to OpenCV BGR image
        try:
            if msg.encoding == 'rgb8':
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding == 'bgr8':
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

        # 3. Object Tracking between frames
        if detections:
            primary_det = max(detections, key=lambda d: d.confidence)
            self.tracker.init(frame, primary_det.bbox)
        else:
            ok, _, _ = self.tracker.update(frame)
            if ok:
                det = self.tracker.to_detection()
                if det:
                    detections = [det]

        # 4. Compute target point message
        target_msg = Point()
        if detections:
            primary = max(detections, key=lambda d: d.confidence)
            target_msg.x = float(primary.center[0])
            target_msg.y = float(primary.center[1])
            target_msg.z = 1.0  # Found
        else:
            target_msg.x = 0.0
            target_msg.y = 0.0
            target_msg.z = -1.0  # Lost

        self.pub_target.publish(target_msg)

        # 5. Render HUD Overlay
        annotated_frame = frame.copy()
        self.overlay.draw(
            annotated_frame,
            detections,
            fps=fps_val,
            vision_mode=self.vision_mode,
            nav_command=f"Target: ({target_msg.x:.0f}, {target_msg.y:.0f})" if target_msg.z > 0 else "Searching..."
        )

        # 6. Publish annotated image
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
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'detector') and node.detector is not None:
            node.detector.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
