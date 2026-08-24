# Smart Car — ROS 2 & Gazebo Simulation Architecture & User Guide

## 1. System Overview

The **Smart Car** simulation provides a complete digital twin of the 4-wheel independent swerve-drive autonomous robot inside **Gazebo Sim 10.4.0** with **ROS 2 Lyrical**.

The simulation is built strictly **around the existing algorithms**:
- Core swerve kinematics ([`navigation/swerve_kinematics.py`](file:///home/diya/smart_car/navigation/swerve_kinematics.py))
- Discrete PID controller with anti-windup ([`navigation/pid.py`](file:///home/diya/smart_car/navigation/pid.py))
- High-level navigation behaviors ([`navigation/navigator.py`](file:///home/diya/smart_car/navigation/navigator.py))
- Vision detectors: Color, ArUco, Lane, YOLOv8-nano, MOSSE Tracker ([`vision/`](file:///home/diya/smart_car/vision/))
- Sci-Fi HUD overlay telemetry ([`display/overlay.py`](file:///home/diya/smart_car/display/overlay.py))

```
                               ┌──────────────────────────────────────────────┐
                               │           Perception (Vision Node)           │
                               │   (Color / YOLO / ArUco / Lane + Tracker)    │
                               └──────────────────────┬───────────────────────┘
                                                      │ /target_point (geometry_msgs/Point)
                                                      ▼
                               ┌──────────────────────────────────────────────┐
                               │           Navigation Controller Node         │
                               │          (Follow / Avoid / Patrol PID)       │
                               └──────────────────────┬───────────────────────┘
                                                      │ /cmd_vel (geometry_msgs/Twist)
                                                      ▼
                               ┌──────────────────────────────────────────────┐
                               │         Swerve Controller Node (ROS 2)       │
                               │     Wraps navigation/swerve_kinematics.py    │
                               │     + Shortest-Path Steering Optimization    │
                               │     + Closed-Loop Odometry & TF Broadcaster  │
                               └──────┬────────────────────────────────┬──────┘
                   4× Steering Angles │                                │ 4× Wheel Angular Velocities
                 (Float64: fl, fr, ...)│                                │ (Float64: fl, fr, ...)
                                      ▼                                ▼
                               ┌──────────────────────────────────────────────┐
                               │               ros_gz_bridge                  │
                               └──────────────────────┬───────────────────────┘
                                                      │ Gazebo Transport
                                                      ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
 │                                           Gazebo Sim 10.4                                              │
 │                                                                                                        │
 │   ┌───────────────────────┐   ┌───────────────────────────────┐   ┌────────────────────────────────┐   │
 │   │     Chassis Link      │   │     4× Swerve Module Pods     │   │       Simulated Sensors        │   │
 │   │ (Mass 2.5 kg, Inertia)│   │ (4 Steer Jnts, 4 Wheel Jnts)  │   │  (IMU 100Hz, Camera 640x480)   │   │
 │   └───────────────────────┘   └───────────────────────────────┘   └────────────────┬───────────────┘   │
 └────────────────────────────────────────────────────────────────────────────────────┼───────────────────┘
                                                                                      │ /camera/image_raw
                                                                                      │ (sensor_msgs/Image)
                                                                                      ▼
                                                                        [Perception Node & RViz2]
```

---

## 2. Package Architecture

```
smart_car/
├── config.py
├── main.py
├── requirements.txt
├── navigation/                     # Reusable Core Mathematics
│   ├── swerve_kinematics.py
│   ├── pid.py
│   └── navigator.py
├── vision/                         # Reusable Detectors & Trackers
│   ├── detector.py
│   ├── color_detector.py
│   ├── aruco_detector.py
│   ├── lane_detector.py
│   ├── yolo_detector.py
│   └── tracker.py
├── display/                        # HUD Overlays
│   └── overlay.py
├── esp32_firmware/                 # Physical Hardware MCU Code (Untouched)
│   ├── platformio.ini
│   └── src/main.cpp
│
└── ros2_ws/src/
    ├── smart_car_description/      # URDF / Xacro Robot Model
    │   ├── urdf/
    │   │   ├── smart_car.urdf.xacro # Main vehicle description
    │   │   ├── swerve_module.xacro  # Parametric swerve pod macro
    │   │   └── sensors.xacro        # Camera and IMU descriptions
    │   ├── rviz/smart_car.rviz     # Pre-configured RViz display
    │   └── launch/view_robot.launch.py
    │
    ├── smart_car_gazebo/           # Gazebo Worlds & Physics
    │   ├── worlds/smart_car_arena.sdf # 6m x 6m testing arena with targets
    │   ├── config/ros_gz_bridge.yaml # Bridge configuration
    │   └── launch/sim.launch.py
    │
    ├── smart_car_control/          # Swerve Kinematics Controller & Odometry
    │   ├── smart_car_control/
    │   │   ├── swerve_controller_node.py # Core controller wrapping kinematics
    │   │   └── teleop_keyboard.py        # WASD keyboard teleoperation
    │   └── launch/control.launch.py
    │
    └── smart_car_ros/              # High-Level Perception, Navigation & Bringup
        ├── smart_car_ros/
        │   ├── vision_node.py      # Multi-mode vision node with tracker & HUD
        │   ├── navigation_node.py  # PID navigation state machine
        │   └── esp32_bridge_node.py # Physical hardware serial bridge
        └── launch/
            ├── smart_car_sim.launch.py   # Master Simulation Bringup
            └── smart_car_real.launch.py  # Master Real Hardware Bringup
```

---

## 3. Physical Robot & Simulation Parameters

| Property | Physical Value | Simulation Representation |
|---|---|---|
| **Chassis Dimensions** | $25\text{ cm} \times 20\text{ cm} \times 8\text{ cm}$ | `<box size="0.25 0.20 0.08"/>` |
| **Chassis Mass** | $2.5\text{ kg}$ | `<mass value="2.5"/>` with accurate inertia matrix |
| **Wheelbase ($L$)** | $0.20\text{ m}$ ($20\text{ cm}$) | Front to rear axle distance |
| **Track Width ($W$)** | $0.15\text{ m}$ ($15\text{ cm}$) | Left to right wheel centerline distance |
| **Wheel Diameter / Radius** | $65\text{ mm}$ / $32.5\text{ mm}$ | Cylinder $r = 0.0325\text{ m}$, width $0.026\text{ m}$ |
| **Camera Sensor** | Pi Camera Module v2 (Sony IMX219) | $640\times 480$ @ 30 FPS, H-FOV $62.2^\circ$ ($1.08559\text{ rad}$) |
| **Module Coordinates** | FL: $(+0.10, +0.075)$, FR: $(+0.10, -0.075)$, RL: $(-0.10, +0.075)$, RR: $(-0.10, -0.075)$ | Defined via `<xacro:swerve_module>` |

---

## 4. Build & Installation Instructions

### Build the ROS 2 Workspace
```bash
cd ~/smart_car/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Launch Commands

### 1. Master Simulation Launch (Complete Autonomous System)
Launches Gazebo Sim arena, spawns the Smart Car, initializes the bridge, launches the swerve controller, vision node, navigation node, and RViz2:
```bash
source /opt/ros/lyrical/setup.bash && source ~/smart_car/ros2_ws/install/setup.bash
ros2 launch smart_car_ros smart_car_sim.launch.py
```

#### Launch Arguments:
- `vision_mode`: Active perception pipeline (`color`, `yolo`, `aruco`, `lane`). Default: `color`.
- `nav_mode`: Autonomous navigation strategy (`follow`, `avoid`, `patrol`, `idle`). Default: `follow`.
- `base_speed`: Target linear speed in m/s. Default: `0.35`.
- `rviz`: Launch RViz2 display (`true` / `false`). Default: `true`.
- `headless`: Run Gazebo without GUI (`true` / `false`). Default: `false`.

*Example: Autonomous YOLO Person Following*:
```bash
ros2 launch smart_car_ros smart_car_sim.launch.py vision_mode:=yolo nav_mode:=follow
```

---

### 2. Manual Keyboard Teleoperation
In a separate terminal, control the robot manually with holonomic WASD + QE keys:
```bash
source /opt/ros/lyrical/setup.bash && source ~/smart_car/ros2_ws/install/setup.bash
ros2 run smart_car_control teleop_keyboard
```

#### Key Controls:
- **W / S**: Move Forward / Reverse ($v_x$)
- **A / D**: Strafe Left / Strafe Right ($v_y$)
- **Q / E**: Rotate Counter-Clockwise / Clockwise ($\omega_z$)
- **Space / K**: Emergency Stop
- **X / Z**: Increase / Decrease Linear Speed ($\pm 0.1\text{ m/s}$)
- **C / V**: Increase / Decrease Angular Speed ($\pm 0.2\text{ rad/s}$)

---

### 3. Visualizing Robot Description Only (RViz2)
```bash
source /opt/ros/lyrical/setup.bash && source ~/smart_car/ros2_ws/install/setup.bash
ros2 launch smart_car_description view_robot.launch.py
```

---

### 4. Physical Hardware Bringup (Real Smart Car)
```bash
source /opt/ros/lyrical/setup.bash && source ~/smart_car/ros2_ws/install/setup.bash
ros2 launch smart_car_ros smart_car_real.launch.py serial_port:=/dev/serial0 vision_mode:=yolo
```

---

## 6. Active ROS 2 Topics Reference

| Topic Name | Message Type | Direction / Role |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | High-level velocity command input to Swerve Controller |
| `/odom` | `nav_msgs/msg/Odometry` | Closed-loop odometry estimated from joint feedback |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic coordinate transforms (`odom -> base_footprint`) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static coordinate transforms (`base_footprint -> base_link -> sensors`) |
| `/joint_states` | `sensor_msgs/msg/JointState` | Real-time position and velocity feedback for all 8 joints |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Raw $640\times 480$ RGB video stream from Gazebo camera |
| `/camera/image_annotated` | `sensor_msgs/msg/Image` | Processed video stream with Sci-Fi HUD overlay, reticle, and bearing |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsic calibration parameters |
| `/imu/data` | `sensor_msgs/msg/Imu` | 6-axis simulated accelerometer and gyroscope data |
| `/target_point` | `geometry_msgs/msg/Point` | Centroid $(x, y)$ of primary vision detection ($z=1.0$ found, $z=-1.0$ lost) |
| `/navigation/set_mode` | `std_msgs/msg/String` | Runtime navigation mode switcher (`idle`, `follow`, `avoid`, `patrol`) |
| `/vision/set_mode` | `std_msgs/msg/String` | Runtime vision mode switcher (`color`, `yolo`, `aruco`, `lane`) |
| `/fl_steer_cmd`, `/fr_steer_cmd`, ... | `std_msgs/msg/Float64` | Steering position commands (radians) to Gazebo joint controllers |
| `/fl_wheel_cmd`, `/fr_wheel_cmd`, ... | `std_msgs/msg/Float64` | Wheel angular velocity commands (rad/s) to Gazebo joint controllers |

---

## 7. Real Hardware vs. Simulation Parity

| Subsystem | Simulation Mode | Real Hardware Mode |
|---|---|---|
| **Camera Feed** | Gazebo Sensor Plugin $\to$ `ros_gz_bridge` $\to$ `/camera/image_raw` | Pi Camera Module v2 / CSI $\to$ `vision_node` / V4L2 |
| **Steering Control** | `swerve_controller_node` $\to$ `/xx_steer_cmd` $\to$ `gz-sim-joint-position-controller` | `esp32_bridge_node` $\to$ UART $\to$ ESP32 `FastAccelStepper` |
| **Drive Control** | `swerve_controller_node` $\to$ `/xx_wheel_cmd` $\to$ `gz-sim-joint-controller` | `esp32_bridge_node` $\to$ UART $\to$ ESP32 LEDC PWM |
| **Perception** | [`vision/detector.py`](file:///home/diya/smart_car/vision/detector.py) + [`vision/tracker.py`](file:///home/diya/smart_car/vision/tracker.py) | **100% Identical Shared Code** |
| **Navigation & PID** | [`navigation/navigator.py`](file:///home/diya/smart_car/navigation/navigator.py) + [`navigation/pid.py`](file:///home/diya/smart_car/navigation/pid.py) | **100% Identical Shared Code** |
| **Kinematics Math** | [`navigation/swerve_kinematics.py`](file:///home/diya/smart_car/navigation/swerve_kinematics.py) | **100% Identical Shared Code** |
