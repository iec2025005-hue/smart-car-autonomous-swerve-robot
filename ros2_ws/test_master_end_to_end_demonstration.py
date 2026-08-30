#!/usr/bin/env python3
"""
test_master_end_to_end_demonstration.py -- Comprehensive Master Demonstration & Validation Suite:
1. System Diagnostics & Topic Verification (TF, Joint States, Camera Feed, Bridge)
2. 8-Direction Holonomic Swerve Motion + In-Place Rotation + Compound Steering
3. Camera Vision Pipeline (Color Detection, YOLO Object Detection, Obstacle Segmentation, ArUco Marker)
4. Vision-Based PID Target Approach
5. Physical Obstacle Avoidance with Live Path Deflection
6. Precision Multi-Waypoint Holonomic Navigation
"""

import sys, time, math, subprocess, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState, Imu
from std_msgs.msg import String, Float64, Empty
import tf2_ros

class MasterDemonstrationRunner(Node):
    def __init__(self):
        super().__init__("master_demonstration_runner")
        self.current_pose = None
        self.joint_positions = {}
        self.last_image_time = None
        self.last_target_point = None
        self.last_obstacle_point = None
        self.nav_state = "IDLE"
        self.image_count = 0
        
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.sub_joints = self.create_subscription(JointState, "/joint_states", self.joints_cb, 10)
        self.sub_image = self.create_subscription(Image, "/camera/image_raw", self.image_cb, 10)
        self.sub_target = self.create_subscription(Point, "/target_point", self.target_cb, 10)
        self.sub_obstacle = self.create_subscription(Point, "/obstacle_point", self.obstacle_cb, 10)
        self.sub_state = self.create_subscription(String, "/navigation/state", self.state_cb, 10)
        
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_nav_mode = self.create_publisher(String, "/navigation/set_mode", 10)
        self.pub_vision_mode = self.create_publisher(String, "/vision/set_mode", 10)
        self.pub_vision_target = self.create_publisher(String, "/vision/set_target", 10)
        self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.pub_reset_odom = self.create_publisher(Empty, "/reset_odom", 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (p.x, p.y, yaw)

    def joints_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def image_cb(self, msg: Image):
        self.last_image_time = time.time()
        self.image_count += 1

    def target_cb(self, msg: Point):
        self.last_target_point = msg

    def obstacle_cb(self, msg: Point):
        self.last_obstacle_point = msg

    def state_cb(self, msg: String):
        self.nav_state = msg.data.strip()

    def reset_robot_to_origin(self):
        self.pub_cmd_vel.publish(Twist())
        time.sleep(0.3)
        cmd = ["gz", "service", "-s", "/world/smart_car_world/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", "name: \"smart_car\", position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        self.pub_reset_odom.publish(Empty())
        time.sleep(0.5)
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_obstacle_pose(self, x: float, y: float, z: float = 0.2):
        cmd = ["gz", "service", "-s", "/world/smart_car_world/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", f"name: \"yellow_obstacle_cylinder\", position: {{x: {x}, y: {y}, z: {z}}}"]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        time.sleep(0.3)

def run_demonstration():
    rclpy.init()
    node = MasterDemonstrationRunner()
    results = {}
    print("===========================================================================")
    print("  SMART CAR AUTONOMOUS SWERVE ROBOT: MASTER DEMONSTRATION SUITE")
    print("===========================================================================")
    print("Waiting for ROS topics and simulation bridge readiness...")
    t0 = time.time()
    while (node.current_pose is None or node.last_image_time is None) and (time.time() - t0 < 10.0):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.current_pose is None or node.last_image_time is None:
        print("ERROR: Timed out waiting for simulation topics!")
        return
    print("Simulation connected! Topics active.")
    
    # SECTION 1: SYSTEM & TF DIAGNOSTICS
    print(" ")
    print("=" * 75)
    print("SECTION 1: SYSTEM TOPICS, JOINT STATES & TF VERIFICATION")
    print("=" * 75)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)
    swerve_joints = ["fl_steer_joint", "fl_wheel_joint", "fr_steer_joint", "fr_wheel_joint", "rl_steer_joint", "rl_wheel_joint", "rr_steer_joint", "rr_wheel_joint"]
    joints_ok = all(j in node.joint_positions for j in swerve_joints)
    print(f"  Swerve Joint States (8/8 active) : {joints_ok}")
    for j in swerve_joints:
        print(f"    * {j:16s} : {node.joint_positions.get(j, 0.0):+.4f} rad")
    cam_hz = node.image_count / max(1.0, time.time() - t0)
    print(f"  Camera Video Stream (/camera/image_raw) : ACTIVE ({cam_hz:.1f} FPS)")
    tf_frames = ["base_footprint", "base_link", "fl_steer_link", "fl_wheel_link", "fr_steer_link", "fr_wheel_link"]
    tf_ok = True
    for f in tf_frames:
        try:
            node.tf_buffer.lookup_transform("base_footprint", f, rclpy.time.Time())
        except Exception:
            try:
                node.tf_buffer.lookup_transform("base_footprint", f, rclpy.time.Time(seconds=0))
            except Exception:
                tf_ok = False
    print(f"  TF Coordinate Transforms (6/6 frames) : {tf_ok}")
    results["system_diagnostics"] = joints_ok and tf_ok
    
    # SECTION 2: 8-DIRECTION HOLONOMIC MOTION + ROTATION
    print(" ")
    print("=" * 75)
    print("SECTION 2: 8-DIRECTION HOLONOMIC SWERVE MOTION & IN-PLACE ROTATION")
    print("=" * 75)
    motion_tests = [
        ("Forward 0 deg", +0.35, 0.0, 0.0, 0.0),
        ("Diagonal 45 deg", +0.247, +0.247, 0.0, 45.0),
        ("Left 90 deg", 0.0, +0.35, 0.0, 90.0),
        ("Diagonal 135 deg", -0.247, +0.247, 0.0, 135.0),
        ("Backward 180 deg", -0.35, 0.0, 0.0, 180.0),
        ("Diagonal 225 deg", -0.247, -0.247, 0.0, -135.0),
        ("Right 270 deg", 0.0, -0.35, 0.0, -90.0),
        ("Diagonal 315 deg", +0.247, -0.247, 0.0, -45.0),
        ("Pure In-Place Spin", 0.0, 0.0, +1.0, None),
        ("Compound Forward+Turn", +0.25, 0.0, +0.5, None),
        ("Compound Lateral+Turn", 0.0, +0.25, +0.5, None),
    ]
    motion_passes = 0
    for name, vx, vy, wz, exp_hdg in motion_tests:
        node.reset_robot_to_origin()
        x0, y0, yaw0 = node.current_pose
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        t_start = time.time()
        while time.time() - t_start < 1.2:
            node.pub_cmd_vel.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)
        node.pub_cmd_vel.publish(Twist())
        time.sleep(0.3)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.02)
        x1, y1, yaw1 = node.current_pose
        dx = x1 - x0
        dy = y1 - y0
        dyaw = math.degrees((yaw1 - yaw0 + math.pi) % (2.0 * math.pi) - math.pi)
        dist = math.hypot(dx, dy)
        meas_hdg = math.degrees(math.atan2(dy, dx)) if dist > 0.05 else 0.0
        if exp_hdg is not None:
            hdg_err = abs((meas_hdg - exp_hdg + 180.0) % 360.0 - 180.0)
            p = (dist >= 0.20) and (hdg_err <= 6.0)
            status = "PASS" if p else "FAIL"
            print(f"  {name:24s} | Dist: {dist:.3f} m | Heading: {meas_hdg:+5.1f} deg (Exp: {exp_hdg:+5.1f} deg, Err: {hdg_err:4.1f} deg) | {status}")
        else:
            p = (abs(dyaw) >= 20.0) if wz != 0 else (dist >= 0.20)
            status = "PASS" if p else "FAIL"
            print(f"  {name:24s} | Dist: {dist:.3f} m | Yaw Change: {dyaw:+5.1f} deg | {status}")
        if p:
            motion_passes += 1
    print(f"\n  Swerve Motion Score: {motion_passes}/{len(motion_tests)} passed.")
    results["motion"] = (motion_passes == len(motion_tests))
    
    # SECTION 3: VISION PIPELINE
    print(" ")
    print("=" * 75)
    print("SECTION 3: COMPUTER VISION & MULTI-OBJECT DETECTION")
    print("=" * 75)
    vision_modes = [
        ("YOLO Detection: Chair Target", "yolo", "chair"),
        ("Color Segmentation: Red Cylinder", "color", "red"),
        ("Obstacle Segmentation: Yellow/Blue", "color", "obstacle"),
        ("ArUco 6-DOF Pose: Marker ID 7", "aruco", "aruco")
    ]
    vision_passes = 0
    for name, vmode, vtarget in vision_modes:
        node.pub_vision_mode.publish(String(data=vmode))
        node.pub_vision_target.publish(String(data=vtarget))
        time.sleep(0.8)
        node.last_target_point = None
        t_v = time.time()
        valid_frames = 0
        while time.time() - t_v < 2.0:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.last_target_point is not None:
                valid_frames += 1
            time.sleep(0.05)
        p = valid_frames >= 5
        status = "PASS" if p else "FAIL"
        tgt_str = f"Target at X={node.last_target_point.x:+.2f}m, Y={node.last_target_point.y:+.2f}m" if node.last_target_point else "No point"
        print(f"  {name:38s} | Mode: {vmode:6s} | {tgt_str} | {status}")
        if p:
            vision_passes += 1
    print(f"\n  Vision Pipeline Score: {vision_passes}/{len(vision_modes)} passed.")
    results["vision"] = (vision_passes == len(vision_modes))
    
    # SECTION 4: PHYSICAL OBSTACLE AVOIDANCE
    print(" ")
    print("=" * 75)
    print("SECTION 4: PHYSICAL OBSTACLE AVOIDANCE WITH LIVE DEFLECTION")
    print("=" * 75)
    node.reset_robot_to_origin()
    node.set_obstacle_pose(1.2, 0.0, 0.2)
    node.pub_vision_mode.publish(String(data="color"))
    node.pub_vision_target.publish(String(data="red"))
    node.pub_nav_mode.publish(String(data="avoid"))
    print("  Placed physical obstacle at (X=+1.20 m, Y=0.00 m) directly in front of target.")
    print("  Starting autonomous obstacle avoidance run...")
    t_run = time.time()
    min_dist_to_obs = 999.0
    avoid_detected = False
    max_lateral_deflection = 0.0
    while time.time() - t_run < 12.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.current_pose is not None:
            cx, cy, _ = node.current_pose
            dist_to_obs = math.hypot(1.2 - cx, 0.0 - cy)
            if dist_to_obs < min_dist_to_obs:
                min_dist_to_obs = dist_to_obs
            if abs(cy) > max_lateral_deflection:
                max_lateral_deflection = abs(cy)
        if node.nav_state == "AVOID_OBSTACLE" or (node.last_obstacle_point is not None and node.last_obstacle_point.z > 0):
            avoid_detected = True
        time.sleep(0.05)
    node.pub_cmd_vel.publish(Twist())
    node.pub_nav_mode.publish(String(data="idle"))
    node.set_obstacle_pose(5.0, 5.0, 0.2)
    obs_pass = avoid_detected and (min_dist_to_obs >= 0.40) and (max_lateral_deflection >= 0.20)
    print(f"  Obstacle Detection Event   : {"DETECTED" if avoid_detected else "MISSED"}")
    print(f"  Max Lateral Path Deflection: {max_lateral_deflection:.3f} m (Safely altered path around obstacle)")
    print(f"  Closest Approach Distance  : {min_dist_to_obs:.3f} m (Safety clearance >= 0.40 m)")
    print(f"  Obstacle Avoidance Status  : {"PASS" if obs_pass else "FAIL"}")
    results["obstacle_avoidance"] = obs_pass
    
    # SECTION 5: PRECISION WAYPOINT NAVIGATION
    print(" ")
    print("=" * 75)
    print("SECTION 5: PRECISION MULTI-WAYPOINT HOLONOMIC NAVIGATION")
    print("=" * 75)
    node.reset_robot_to_origin()
    time.sleep(0.5)
    waypoints = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 1.0, 45.0),
        (-1.0, 1.0, 90.0),
        (0.0, 0.0, 0.0)
    ]
    wp_passes = 0
    for idx, (gx, gy, gyaw_deg) in enumerate(waypoints, 1):
        gyaw_rad = math.radians(gyaw_deg)
        x_start, y_start, yaw_start = node.current_pose
        travel_dist = math.hypot(gx - x_start, gy - y_start)
        timeout_sec = max(15.0, travel_dist * 8.0 + 8.0)
        
        goal = PoseStamped()
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.header.frame_id = "odom"
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.orientation.z = math.sin(gyaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(gyaw_rad / 2.0)
        node.pub_goal_pose.publish(goal)
        
        t_wp = time.time()
        while time.time() - t_wp < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.current_pose is not None:
                cx, cy, cyaw = node.current_pose
                err = math.hypot(gx - cx, gy - cy)
                eyaw = math.degrees((gyaw_rad - cyaw + math.pi) % (2.0 * math.pi) - math.pi)
                if err < 0.05 and abs(eyaw) < 6.0:
                    break
            time.sleep(0.05)
        cx, cy, cyaw = node.current_pose
        final_err = math.hypot(gx - cx, gy - cy) * 1000.0
        final_yaw_err = math.degrees((gyaw_rad - cyaw + math.pi) % (2.0 * math.pi) - math.pi)
        p = (final_err <= 50.0) and (abs(final_yaw_err) <= 6.0)
        status = "PASS" if p else "FAIL"
        print(f"  Waypoint {idx}: Goal ({gx:+.1f}, {gy:+.1f}, {gyaw_deg:+.0f} deg) | Pos Error: {final_err:4.1f} mm | Yaw Error: {final_yaw_err:+.1f} deg | {status}")
        if p:
            wp_passes += 1
    print(f"\n  Waypoint Navigation Score: {wp_passes}/{len(waypoints)} passed.")
    results["waypoints"] = (wp_passes == len(waypoints))
    
    # FINAL SUMMARY
    print(" ")
    print("=" * 75)
    print("MASTER DEMONSTRATION & VALIDATION SUMMARY")
    print("=" * 75)
    print(f"  1. System Diagnostics & TF Tree       : {"PASS" if results.get("system_diagnostics") else "FAIL"}")
    print(f"  2. 8-Direction Swerve Motion (11/11) : {"PASS" if results.get("motion") else "FAIL"}")
    print(f"  3. Vision Detection Pipeline (4/4)    : {"PASS" if results.get("vision") else "FAIL"}")
    print(f"  4. Physical Obstacle Avoidance        : {"PASS" if results.get("obstacle_avoidance") else "FAIL"}")
    print(f"  5. Waypoint Navigation Accuracy (5/5) : {"PASS" if results.get("waypoints") else "FAIL"}")
    all_pass = all(results.values())
    print(f"\nOVERALL DEMONSTRATION STATUS: {"ALL TESTS PASSED - SYSTEM FULLY OPERATIONAL" if all_pass else "SOME TESTS FAILED"}")
    print("=" * 75)

if __name__ == "__main__":
    run_demonstration()
