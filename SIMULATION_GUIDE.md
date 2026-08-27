# Smart Car ROS 2 & Gazebo Simulation Guide

**Environment**: Ubuntu 26.04 LTS  
**ROS Distribution**: ROS 2 Lyrical  
**Physics Engine**: Gazebo Sim 10.4.0 (DART Physics)  
**CAD Model Ground Truth**: Fusion 360 Solid Bodies (`cad/4_WheelsSwerve01.stp`)  

---

## 1. System Architecture & Information Flow

```
                 +-------------------------------------------------------+
                 |            Gazebo Camera (/camera/image_raw)          |
                 +---------------------------+---------------------------+
                                             |
                                             v
                       +-----------------------------------+
                       |    vision_node (Perception)       |
                       |  - Color / ArUco / Lane / YOLO    |
                       |  - MOSSE Object Tracker           |
                       |  - Real-Time HUD Overlay          |
                       +-----------------+-----------------+
                                         |
                       /target_point     |  /obstacle_point
                       (Target center)   |  (Obstacle center & width)
                                         v
                       +-----------------------------------+
                       |  navigation_node (Planning & PID) |
                       |  - Follow / Avoid / Patrol Modes  |
                       |  - Autonomous Obstacle Avoidance  |
                       |  - Continuous PID Error Reduction |
                       +-----------------+-----------------+
                                         |
                       /cmd_vel          |  /pid/diagnostics, /navigation/state
                       (Target Twist)    |  (PID terms, active state)
                                         v
                       +-----------------------------------+
                       |    swerve_controller_node         |
                       |  - SwerveKinematics (4-wheel)     |
                       |  - Modulo Shortest-Path Unwrapping|
                       |  - Cosine Velocity Scaling        |
                       |  - Odometry & TF (/odom, /tf)     |
                       +-----------------+-----------------+
                                         |
                          8 Joint Cmds   |  /fl_steer_cmd, /fl_wheel_cmd, ...
                                         v
                 +-------------------------------------------------------+
                 |            Gazebo Sim 10.4.0 (Real CAD Model)         |
                 |  - 4 Independent Steering Continuous Steppers (Z-axis)|
                 |  - 4 Independent Wheel Drive Actuators (Y-axis)       |
                 |  - Clearance-Aligned Chassis Deck Bounding Box        |
                 +-------------------------------------------------------+
```

---

## 2. Launching the Simulation

### A. Full Launch with GUI & RViz2 (Default YOLO Chair Mode)
```bash
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/iron/setup.bash 2>/dev/null || true
source /home/diya/smart_car/ros2_ws/install/setup.bash
ros2 launch smart_car_ros smart_car_sim.launch.py nav_mode:=follow vision_mode:=yolo target_class:=chair
```

### B. Headless Launch (Fast Benchmarking / Automated Testing)
```bash
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/iron/setup.bash 2>/dev/null || true
source /home/diya/smart_car/ros2_ws/install/setup.bash
ros2 launch smart_car_ros smart_car_sim.launch.py nav_mode:=idle vision_mode:=yolo target_class:=chair headless:=true rviz:=false
```


---

## 3. Topics & Telemetry Reference

| Topic Name | Message Type | Description |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Live raw video stream from Gazebo camera ($640\times 480$ at $30\text{ FPS}$). |
| `/camera/image_annotated` | `sensor_msgs/msg/Image` | Video stream with real-time HUD bounding boxes, FPS, and telemetry. |
| `/target_point` | `geometry_msgs/msg/Point` | Detected target centroid $(x, y)$ and bounding box width $(z)$. |
| `/obstacle_point` | `geometry_msgs/msg/Point` | Detected obstacle centroid $(x, y)$ and bounding box width $(z)$. |
| `/pid/diagnostics` | `std_msgs/msg/Float64MultiArray` | Array of `[error, p_term, i_term, d_term, output, vx_cmd, vy_cmd]`. |
| `/navigation/state` | `std_msgs/msg/String` | Active state machine state (`FOLLOW`, `AVOID`, `PATROL`, `IDLE`). |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Commanded body twist $(v_x, v_y, \omega_z)$. |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry computed from joint feedback. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms (`odom -> base_footprint -> base_link`). |

---

## 4. Verification & Testing Scripts

### 1. Autonomous Obstacle Avoidance & Target Follow Closed Loop
```bash
python3 /home/diya/smart_car/ros2_ws/test_autonomous_obstacle_avoidance.py
```

### 2. 8-Point Swerve Motion Benchmark
```bash
python3 /home/diya/smart_car/ros2_ws/run_swerve_benchmark.py
```

### 3. Perception Detectors Matrix Test
```bash
python3 /home/diya/smart_car/scratch/test_vision_detectors.py
```
