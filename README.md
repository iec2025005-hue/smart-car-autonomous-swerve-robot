# Smart Car — Autonomous 4-Wheel Independent Swerve Robot

**Production Embedded System**: Raspberry Pi 4 (Ubuntu/Debian) + ESP32 DevKit V1  
**Digital Twin & Simulation**: ROS 2 Lyrical + Gazebo Sim 10.4.0 (DART Physics)  
**CAD Model Source of Truth**: Fusion 360 Solid Bodies (`cad/4_WheelsSwerve01.stp`)  

---

## 1. Project Overview & Architecture

The Smart Car is a vision-guided 4-wheel independent swerve-drive robot. Each swerve pod has 2 degrees of freedom:
1. **Steering Axis ($Z$)**: Continuous $360^\circ$ orientation driven by high-torque stepper motors.
2. **Drive Axis ($Y$)**: Continuous rolling traction driven by geared DC motors with optical encoders.

```
+-----------------------------------------------------------------------------------------+
|                                REAL HARDWARE ARCHITECTURE                               |
+-----------------------------------------------------------------------------------------+
Pi Camera Module v2 (CSI IMX219)
       │ (RGB888 -> BGR, 640x480 @ 30 FPS)
       ▼
Vision Pipeline (YOLO / Color / ArUco / Lane)
       │ Detections
       ▼
Object Tracker (MOSSE / KCF / CSRT)
       │ Centroids & Bounding Boxes
       ▼
Navigator (Follow / Avoid / Patrol / Idle) + PID Controller
       │ vx, vy, omega
       ▼
Swerve Kinematics (navigation/swerve_kinematics.py)
       │ "SWERVE:a_fr,s_fr,a_fl,s_fl,a_rl,s_rl,a_rr,s_rr\n" (UART 115200)
       ▼
ESP32 DevKit V1 (esp32_firmware/src/main.cpp)
       ├── 4× DRV8825 Stepper Motors (Independent Pod Steering)
       └── 4× DC Motors (Independent Pod Drive + Encoders)
```

```
+-----------------------------------------------------------------------------------------+
|                                ROS 2 & GAZEBO SIMULATION                                |
+-----------------------------------------------------------------------------------------+
Gazebo Sim Camera Sensor (smart_car_arena.sdf)
       │ gz.msgs.Image
       ▼
ros_gz_bridge -> /camera/image_raw (sensor_msgs/Image)
       │ cv_bridge -> BGR NumPy array
       ▼
smart_car_ros/vision_node (Directly executes vision/detector.py & tracker.py)
       │ /target_point (geometry_msgs/Point) & /camera/image_annotated
       ▼
smart_car_ros/navigation_node (Directly wraps navigation/navigator.py & pid.py)
       │ /cmd_vel (geometry_msgs/Twist)
       ▼
smart_car_control/swerve_controller_node (Directly executes navigation/swerve_kinematics.py)
       │ 4× Steer Position & 4× Wheel Velocity Commands (Float64)
       ▼
ros_gz_bridge -> Gazebo Sim Joint Position & Velocity Controllers
       │ Actuation
       ▼
Gazebo Simulated 4-Wheel Swerve Robot (smart_car.urdf.xacro)
```

---

## 2. Hardware Specifications & Kinematic Dimensions

All dimensions match the physical Fusion 360 CAD (`cad/4_WheelsSwerve01.stp`):

| Parameter | CAD / Physical Value | Description |
|---|:---:|---|
| **Wheelbase ($L$)** | `0.208 m` ($208\text{ mm}$) | Longitudinal distance between front and rear steering axes. |
| **Track Width ($W$)** | `0.2285 m` ($228.5\text{ mm}$) | Transverse distance between left and right steering axes. |
| **Wheel Radius ($R$)** | `0.039965 m` ($39.965\text{ mm}$) | Outer radius of the physical drive wheels. |
| **Wheel Diameter ($D$)** | `0.07993 m` ($79.93\text{ mm}$) | Outer diameter of the physical drive wheels. |
| **Wheel Width ($W_w$)** | `0.03765 m` ($37.65\text{ mm}$) | Thickness/width of the physical drive wheels. |
| **Hub Bore Diameter** | `0.00484 m` ($4.84\text{ mm}$) | Center hub axle hole diameter. |
| **Steering-to-Wheel Offset** | `0.00 mm` | Vertical steering axis intersects wheel axle concentrically. |
| **Steering Motors** | 4× NEMA 17 / DRV8825 | $1.8^\circ$ step angle ($200\text{ steps/rev}$) with FastAccelStepper. |
| **Drive Motors** | 4× Geared DC Motors | 12V DC with TB6612FNG dual H-bridges and optical encoders. |
| **Host SBC** | Raspberry Pi 4 (4GB/8GB) | High-level perception, planning, and web streaming. |
| **Embedded Controller** | ESP32 DevKit V1 | Real-time motor timing, step pulse generation, and PID velocity control. |

---

## 3. ROS 2 Packages Overview

* **`smart_car_description`**: URDF/Xacro models, corner-specific CAD STL meshes (`fl_steer_bracket.stl`, `fr_steer_bracket.stl`, `rl_steer_bracket.stl`, `rr_steer_bracket.stl`, `wheel_left.stl`, `wheel_right.stl`, `chassis.stl`), Gazebo joint controller plugins, and RViz configuration.
* **`smart_car_control`**: `swerve_controller_node` wrapping [`navigation/swerve_kinematics.py`](navigation/swerve_kinematics.py) with continuous modulo angle unwrapping, cosine velocity scaling, odometry computation, and `/odom` + `/tf` broadcast.
* **`smart_car_gazebo`**: Simulation launch files, materials, and the arena world (`smart_car_arena.sdf`) with visual landmarks and ground physics.
* **`smart_car_ros`**: `vision_node` (OpenCV perception + MOSSE tracker) and `navigation_node` (wraps [`navigation/navigator.py`](navigation/navigator.py) and [`navigation/pid.py`](navigation/pid.py)).

---

## 4. Vision & Navigation Subsystems

### Perception Modes (`/vision/set_mode`)
1. **`color`**: Dual HSV red thresholding with morphology and contour area filtering.
2. **`aruco`**: OpenCV ArUco detector using `DICT_4X4_50` dictionary.
3. **`lane`**: Perspective transform (birds-eye view), white/yellow thresholding, histogram sliding window, and polynomial lane center tracking.
4. **`yolo`**: Ultralytics YOLOv8-nano (`models/yolov8n.pt`) with class filtering.

### Navigation Modes (`/navigation/set_mode`)
1. **`follow`**: Drives forward while using closed-loop PID error correction to center the target in frame.
2. **`avoid`**: Holonomic lateral strafe away from the highest-confidence obstacle.
3. **`patrol`**: Forward transit with automatic emergency stopping when obstacles enter close range.
4. **`idle`**: Complete motor stop; vision pipeline continues streaming.

---

## 5. Build & Launch Instructions

### Prerequisites (Ubuntu 26.04 LTS / ROS 2 Lyrical)
```bash
source /opt/ros/lyrical/setup.bash
```

### Build Workspace
```bash
cd /home/diya/smart_car/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Simulation
```bash
# Launch complete simulation with GUI, RViz, and perception
ros2 launch smart_car_ros smart_car_sim.launch.py

# Launch headless simulation for fast benchmarking / automated tests
ros2 launch smart_car_ros smart_car_sim.launch.py headless:=true rviz:=false
```

---

## 6. Testing & Validation Commands

### 1. Execute 8-Point Swerve Motion Benchmark (Tests A through H)
```bash
python3 /home/diya/smart_car/ros2_ws/run_swerve_benchmark.py
```

### 2. Test Vision Detectors Unit Suite
```bash
python3 /home/diya/smart_car/tests/test_vision_detectors.py
```

### 3. Test Web Teleoperation & Live Stream
```bash
python3 /home/diya/smart_car/web/app.py
# Access via browser at http://localhost:5000
```

---

## 7. Documentation Index

* [`FINAL_PROJECT_STATUS.md`](FINAL_PROJECT_STATUS.md): Complete feature audit and test classifications across all subsystems.
* [`SWERVE_TEST_RESULTS.md`](SWERVE_TEST_RESULTS.md): Numerical trajectory, joint angle, and wheel velocity logs for tests A through H.
* [`HARDWARE_SETUP.md`](HARDWARE_SETUP.md): Electrical wiring, ESP32 pinout, motor drivers, encoders, and UART serial protocol.
* [`SIMULATION_GUIDE.md`](SIMULATION_GUIDE.md): Detailed ROS 2 & Gazebo Sim user manual with runtime topics and teleoperation examples.
