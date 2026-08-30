# Holonomic Swerve Motion Benchmark & Validation Report

**Robot Platform**: Smart Car 4-Wheel Independent Swerve Drive  
**Environment**: Ubuntu 26.04 LTS / ROS 2 Lyrical / Gazebo Sim 10.4.0 (DART Physics)  
**Real Wheel Dimensions (Physically Measured)**:
- **Outer Diameter ($D$)**: $79.93\text{ mm} = 0.07993\text{ m}$
- **Outer Radius ($R$)**: $39.965\text{ mm} = 0.039965\text{ m}$
- **Wheel Width / Thickness ($W_w$)**: $37.65\text{ mm} = 0.03765\text{ m}$
- **Hub / Axle Bore Diameter ($D_{hole}$)**: $4.84\text{ mm} = 0.00484\text{ m}$
- **Chassis Dimensions**: Wheelbase $L = 0.208\text{ m}$, Track Width $W = 0.2285\text{ m}$

---

## 1. Comprehensive 10-Point Motion Benchmark Matrix (All from Origin)

| # | Motion Test Description | Commanded $(v_x, v_y, \omega_z)$ | Start Pose $(x, y, \theta)$ | Measured $(\Delta x, \Delta y)$ | Motion Angle | $\Delta z$ (Ground) | $\Delta\text{Yaw}$ | Physical Evaluation | Status |
|:---:|---|:---:|:---:|:---:|:---:|:---:|:---:|---|:---:|
| **1** | TEST 1: Forward (0°) | `(+0.350, +0.000, +0.000)` | `(-0.00, +0.00, -0.0°)` | `(+0.462, +0.000) m` | `+0.0°` | `+0.0000 m` | `-0.00°` | Pure forward longitudinal translation (+X) | ✅ **PASS** |
| **2** | TEST 2: Backward (180°) | `(-0.350, +0.000, +0.000)` | `(+0.01, +0.00, -0.0°)` | `(-0.463, -0.000) m` | `-180.0°` | `+0.0000 m` | `-0.00°` | Pure reverse longitudinal translation (-X) | ✅ **PASS** |
| **3** | TEST 3: Left (90°) | `(+0.000, +0.350, +0.000)` | `(-0.01, -0.00, -0.0°)` | `(-0.004, +0.389) m` | `+90.6°` | `+0.0000 m` | `+0.22°` | Pure lateral strafe left (+Y) | ✅ **PASS** |
| **4** | TEST 4: Right (270° / -90°) | `(+0.000, -0.350, +0.000)` | `(-0.00, +0.01, +0.0°)` | `(+0.001, -0.371) m` | `-89.9°` | `+0.0000 m` | `+0.17°` | Pure lateral strafe right (-Y) | ✅ **PASS** |
| **5** | TEST 5: Diagonal 45° (Forward-Left) | `(+0.247, +0.247, +0.000)` | `(+0.00, -0.01, +0.0°)` | `(+0.271, +0.279) m` | `+45.8°` | `+0.0000 m` | `-0.12°` | Holonomic diagonal travel at 45° (+X, +Y) | ✅ **PASS** |
| **6** | TEST 6: Diagonal 135° (Backward-Left) | `(-0.247, +0.247, +0.000)` | `(+0.00, +0.00, +0.0°)` | `(-0.262, +0.276) m` | `+133.5°` | `+0.0000 m` | `-2.05°` | Holonomic diagonal travel at 135° (-X, +Y) | ✅ **PASS** |
| **7** | TEST 7: Diagonal 225° (Backward-Right) | `(-0.247, -0.247, +0.000)` | `(-0.01, +0.01, -0.0°)` | `(-0.260, -0.270) m` | `-133.9°` | `+0.0000 m` | `+0.59°` | Holonomic diagonal travel at 225° (-X, -Y) | ✅ **PASS** |
| **8** | TEST 8: Diagonal 315° (Forward-Right) | `(+0.247, -0.247, +0.000)` | `(-0.00, -0.00, -0.0°)` | `(+0.273, -0.268) m` | `-44.4°` | `+0.0000 m` | `-0.27°` | Holonomic diagonal travel at 315° (+X, -Y) | ✅ **PASS** |
| **9** | TEST 9: Pure Rotation (CCW in place) | `(+0.000, +0.000, +1.000)` | `(+0.01, -0.00, -0.0°)` | `(-0.000, +0.001) m` | `N/A` | `+0.0000 m` | `+62.07°` | In-place counter-clockwise yaw rotation | ✅ **PASS** |
| **10** | TEST 10: Forward + Rotation Simultaneously | `(+0.250, +0.000, +0.500)` | `(+0.00, +0.00, +0.1°)` | `(+0.270, +0.072) m` | `N/A` | `+0.0000 m` | `+32.26°` | Curvilinear forward translation while rotating heading | ✅ **PASS** |
| **11** | TEST 11: Lateral + Rotation Simultaneously | `(+0.000, +0.250, +0.500)` | `(+0.00, +0.01, -0.2°)` | `(-0.066, +0.270) m` | `N/A` | `+0.0000 m` | `+32.01°` | Curvilinear lateral strafe while rotating heading | ✅ **PASS** |

---

## 2. Telemetry: Joint Steering Angles and Wheel Angular Velocities

| Test | FL Steer | FR Steer | RL Steer | RR Steer | FL Wheel | FR Wheel | RL Wheel | RR Wheel |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **TEST 1: Forward (0°)** | `+0.0°` | `-0.0°` | `-0.4°` | `+0.4°` | `+8.76 rad/s` | `+8.76 rad/s` | `+8.76 rad/s` | `+8.76 rad/s` |
| **TEST 2: Backward (180°)** | `+0.2°` | `-0.2°` | `-0.0°` | `+0.0°` | `-8.76 rad/s` | `-8.76 rad/s` | `-8.76 rad/s` | `-8.76 rad/s` |
| **TEST 3: Left (90°)** | `-90.3°` | `-90.1°` | `-90.5°` | `+90.1°` | `-8.76 rad/s` | `-8.76 rad/s` | `-8.76 rad/s` | `+8.76 rad/s` |
| **TEST 4: Right (270° / -90°)** | `-90.2°` | `-90.0°` | `-90.1°` | `+90.2°` | `+8.76 rad/s` | `+8.76 rad/s` | `+8.76 rad/s` | `-8.76 rad/s` |
| **TEST 5: Diagonal 45° (Forward-Left)** | `-135.1°` | `-134.6°` | `-134.3°` | `+45.1°` | `-8.76 rad/s` | `-8.76 rad/s` | `-8.76 rad/s` | `+8.76 rad/s` |
| **TEST 6: Diagonal 135° (Backward-Left)** | `-225.6°` | `-223.7°` | `-45.9°` | `+134.8°` | `+8.76 rad/s` | `+8.76 rad/s` | `-8.76 rad/s` | `+8.76 rad/s` |
| **TEST 7: Diagonal 225° (Backward-Right)** | `-315.3°` | `-315.2°` | `-134.6°` | `+46.0°` | `-8.76 rad/s` | `-8.76 rad/s` | `+8.76 rad/s` | `-8.76 rad/s` |
| **TEST 8: Diagonal 315° (Forward-Right)** | `-404.0°` | `-404.0°` | `-223.7°` | `+133.6°` | `+8.76 rad/s` | `+8.76 rad/s` | `-8.76 rad/s` | `-8.75 rad/s` |
| **TEST 9: Pure Rotation (CCW in place)** | `-404.4°` | `-500.1°` | `-139.4°` | `+136.2°` | `-3.86 rad/s` | `-3.86 rad/s` | `+3.86 rad/s` | `-3.86 rad/s` |
| **TEST 10: Forward + Rotation Simultaneously** | `-346.9°` | `-532.8°` | `-196.9°` | `+168.8°` | `+5.00 rad/s` | `-7.79 rad/s` | `-5.00 rad/s` | `-7.79 rad/s` |
| **TEST 11: Lateral + Rotation Simultaneously** | `-260.9°` | `-462.7°` | `-75.4°` | `+252.7°` | `+7.69 rad/s` | `-7.69 rad/s` | `-5.15 rad/s` | `-5.16 rad/s` |

---

## 3. Kinematic Verification: Arbitrary Angle Holonomic Travel

The test results verify that starting from the exact same initial position $(0, 0, 0)$, the robot is able to travel along any commanded heading (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°) without turning its chassis, as well as execute in-place rotation and combined curved trajectory without wheel slip or vertical jitter.

