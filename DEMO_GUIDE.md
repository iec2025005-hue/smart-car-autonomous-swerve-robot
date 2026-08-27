# Smart Car — Complete Demonstration & Operator Guide

**Robot Platform**: Autonomous 4-Wheel Independent Swerve-Drive Smart Car  
**Environment**: Ubuntu 26.04 LTS / ROS 2 (Lyrical/Jazzy/Humble) / Gazebo Sim 10.4.0 (DART Physics)  
**CAD Model Ground Truth**: Fusion 360 Solid Bodies (`cad/4_WheelsSwerve01.stp`)  

---

## 1. Quick Start: Launching the Complete Simulation

Open a terminal and run:

```bash
# 1. Source ROS 2 & Workspace
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/iron/setup.bash 2>/dev/null || true
source /home/diya/smart_car/ros2_ws/install/setup.bash

# 2. Launch Complete Simulation (Gazebo GUI, Real CAD Robot, RViz2, Perception & Navigation)
# Default starts in YOLO chair tracking mode:
ros2 launch smart_car_ros smart_car_sim.launch.py nav_mode:=follow vision_mode:=yolo target_class:=chair
```

---

## 2. Dynamic Target Selection & Autonomous Demonstration Commands

### Exact YOLO Chair Target Selection Command
To select `chair` (or switch targets dynamically at runtime):
```bash
ros2 topic pub --once /vision/set_target std_msgs/msg/String "{data: 'chair'}"
```

### Exact Autonomous Chair Mission & Obstacle Avoidance Command
To execute the automated end-to-end mission verification test with zero manual `/cmd_vel` intervention:
```bash
python3 /home/diya/smart_car/ros2_ws/test_autonomous_obstacle_avoidance.py
```

### Complete 5-Demonstration Validation Suite Command
To run all 5 demonstration modes back-to-back:
```bash
python3 /home/diya/smart_car/ros2_ws/test_yolo_and_all_demos.py
```

---

## 3. Step-by-Step Demonstration Scripts (DEMO 1 through DEMO 8)


### DEMO 1: Manual Holonomic Swerve Driving
* **Objective**: Demonstrate 8-DOF independent swerve motion (forward, reverse, pure lateral strafe, in-place rotation, and combined curvilinear arcs).
* **Command**:
  ```bash
  # Forward Drive (0.35 m/s)
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.35, y: 0.0, z: 0.0}}"

  # Pure Lateral Strafe Left (0.35 m/s)
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.35, z: 0.0}}"

  # In-Place CCW Rotation (1.0 rad/s)
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {x: 0.0, y: 0.0, z: 1.0}}"
  ```
* **On-Screen Evidence**:
  - All 4 steering pods reorient to exact kinematic angles.
  - Robot translates sideways with zero chassis rotation or rotates in-place with zero translational drift.
  - Continuous ground contact ($\Delta z = 0.0000\text{ m}$).

---

### DEMO 2: Autonomous Color Target Following
* **Objective**: Camera perceives the red target cylinder in real time, computes pixel error, applies PID control, and drives the swerve modules to center and approach the target.
* **Command**:
  ```bash
  ros2 topic pub --once /vision/set_mode std_msgs/msg/String "{data: 'color'}"
  ros2 topic pub --once /navigation/set_mode std_msgs/msg/String "{data: 'follow'}"
  ```
* **On-Screen Evidence**:
  - `/camera/image_annotated` displays a green bounding box around the red target cylinder with real-time FPS and target coordinates.
  - Car accelerates forward and strafes laterally to keep target at pixel $x = 320\text{ px}$.
  - Stops cleanly when approaching the target destination distance.

---

### DEMO 3: Target Following with Autonomous Obstacle Avoidance
* **Objective**: Demonstrate collision-free navigation where a yellow obstacle cylinder directly blocks the line of sight between the robot and the red target.
* **Command**:
  ```bash
  python3 /home/diya/smart_car/ros2_ws/test_autonomous_obstacle_avoidance.py
  ```
* **On-Screen Evidence**:
  - Robot detects the yellow obstacle cylinder in its path.
  - Swerves laterally to the right with $0.40\text{ m}$ safety clearance.
  - Clears the obstacle without touching it (**Collision count = 0**).
  - Reacquires the red target cylinder and approaches destination.

---

### DEMO 4: ArUco Marker Navigation
* **Objective**: 6-DOF ArUco dictionary recognition (`DICT_4X4_50`, Marker ID 7) and visual pose alignment.
* **Command**:
  ```bash
  ros2 topic pub --once /vision/set_mode std_msgs/msg/String "{data: 'aruco'}"
  ros2 topic pub --once /navigation/set_mode std_msgs/msg/String "{data: 'follow'}"
  ```
* **On-Screen Evidence**:
  - `/camera/image_annotated` renders the detected ArUco marker boundary, corner axes, and ID label `ArUco-7`.
  - Swerve drive navigates directly toward the ArUco marker board.

---

### DEMO 5: Road Lane Following
* **Objective**: Dual white lane boundary detection via perspective transform, Canny edge detection, Hough lines, and polynomial centering.
* **Command**:
  ```bash
  ros2 topic pub --once /vision/set_mode std_msgs/msg/String "{data: 'lane'}"
  ros2 topic pub --once /navigation/set_mode std_msgs/msg/String "{data: 'follow'}"
  ```
* **On-Screen Evidence**:
  - Perspective birds-eye overlay highlights left and right lane boundaries.
  - PID steering keeps the robot centered between the lane lines.

---

### DEMO 6: Ultralytics YOLOv8-nano Object Detection
* **Objective**: Neural network object detection (`models/yolov8n.pt`) with real-time class classification.
* **Command**:
  ```bash
  ros2 topic pub --once /vision/set_mode std_msgs/msg/String "{data: 'yolo'}"
  ```
* **On-Screen Evidence**:
  - `/camera/image_annotated` renders YOLO bounding boxes with class labels and confidence scores.

---

### DEMO 7: Real-Time MOSSE Object Tracking
* **Objective**: High-speed correlation tracker tracking objects across frames between neural detector inferences.
* **Command**:
  ```bash
  python3 /home/diya/smart_car/scratch/test_vision_detectors.py
  ```
* **On-Screen Evidence**:
  - Tracker tracks target continuously across 30+ frames without drift.

---

### DEMO 8: Full Autonomous Closed-Loop Demonstration
* **Objective**: Full end-to-end autonomous mission combining camera perception, obstacle avoidance, PID tracking, swerve kinematics, odometry, and TF broadcast.
* **Command**:
  ```bash
  ros2 launch smart_car_ros smart_car_sim.launch.py nav_mode:=follow vision_mode:=color
  ```
* **On-Screen Evidence**:
  - Complete live mission: perception $\to$ detection $\to$ avoidance $\to$ target approach $\to$ destination stop.
  - Zero manual velocity commands during execution.

---

## 3. Real-Time Telemetry & Topic Inspection

```bash
# Monitor live PID error, terms, and output
ros2 topic echo /pid/diagnostics

# Monitor current navigation state machine mode
ros2 topic echo /navigation/state

# Monitor visual target coordinates
ros2 topic echo /target_point

# Monitor visual obstacle coordinates
ros2 topic echo /obstacle_point

# Monitor wheel odometry
ros2 topic echo /odom
```
