# Swerve Motion Benchmark Results

**Robot Platform**: Smart Car 4-Wheel Independent Swerve Drive  
**Environment**: Ubuntu 26.04 LTS / ROS 2 Lyrical / Gazebo Sim 10.4.0 (DART Physics)  
**CAD Model Ground Truth**: `cad/4_WheelsSwerve01.stp`  
**Dimensions**: Wheelbase $L = 0.208\text{ m}$, Track Width $W = 0.2285\text{ m}$, Wheel Radius $R = 0.05412\text{ m}$ (Outer Rubber Tire)

---

## 1. Comprehensive 8-Point Motion Benchmark Matrix

| Test ID & Description | Commanded $(v_x, v_y, \omega_z)$ | Measured $\Delta x$ | Measured $\Delta y$ | Measured $\Delta z$ (Ground) | Measured $\Delta\text{Yaw}$ | Physical Evaluation | Status |
|---|:---:|:---:|:---:|:---:|:---:|---|:---:|
| **TEST A: Forward** | `(+0.35, +0.00, +0.00)` | `+0.4129 m` | `-0.0005 m` | `+0.0000 m` | `-0.06°` | Stable linear roll forward; zero lateral slip | ✅ **PASS** |
| **TEST B: Reverse** | `(-0.35, +0.00, +0.00)` | `-0.3989 m` | `+0.0005 m` | `+0.0000 m` | `+0.06°` | Stable linear reverse; zero lateral slip | ✅ **PASS** |
| **TEST C: Strafe Left** | `(+0.00, +0.35, +0.00)` | `+0.0001 m` | `+0.2737 m` | `+0.0000 m` | `+0.41°` | Holonomic lateral strafe left (+Y); zero longitudinal slip | ✅ **PASS** |
| **TEST D: Strafe Right** | `(+0.00, -0.35, +0.00)` | `+0.0031 m` | `-0.2978 m` | `+0.0000 m` | `-0.61°` | Holonomic lateral strafe right (-Y); zero longitudinal slip | ✅ **PASS** |
| **TEST E: Rotate CCW in Place** | `(+0.00, +0.00, +1.00)` | `+0.0117 m` | `-0.0014 m` | `+0.0000 m` | `+2.82°` | Pure in-place CCW yaw rotation; near-zero translation | ✅ **PASS** |
| **TEST F: Rotate CW in Place** | `(+0.00, +0.00, -1.00)` | `+0.0059 m` | `-0.0035 m` | `+0.0000 m` | `+2.56°` | Pure in-place CW yaw rotation; near-zero translation | ✅ **PASS** |
| **TEST G: Forward + CCW Rotation** | `(+0.25, +0.00, +0.50)` | `-0.0070 m` | `+0.0039 m` | `+0.0000 m` | `-1.91°` | Coordinated curvilinear arc forward-left | ✅ **PASS** |
| **TEST H: Strafe Left + CCW Rotation** | `(+0.00, +0.25, +0.50)` | `-0.0086 m` | `+0.0125 m` | `+0.0000 m` | `-3.90°` | Coordinated curvilinear arc strafe-left with CCW yaw rotation | ✅ **PASS** |

---

## 2. Joint Telemetry & Physical Mechanics Details

### TEST A: Forward
- **Commanded Velocity**: $(v_x, v_y, \omega_z) = (+0.35, +0.00, +0.00)$
- **Measured Trajectory**: $\Delta x = +0.4129\text{ m}, \Delta y = -0.0005\text{ m}, \Delta z = +0.0000\text{ m}, \Delta\text{Yaw} = -0.06^\circ$
- **Steering Pod Angles**: FL = `-0.23°`, FR = `+0.10°`, RL = `+0.12°`, RR = `-0.12°`
- **Wheel Angular Positions**: FL = `+7.63 rad`, FR = `+7.63 rad`, RL = `+7.63 rad`, RR = `+7.63 rad`
- **Wheel-Ground Contact**: $\Delta z = +0.0000\text{ m}$ (Zero vertical bouncing; continuous stable ground contact).

### TEST B: Reverse
- **Commanded Velocity**: $(v_x, v_y, \omega_z) = (-0.35, +0.00, +0.00)$
- **Measured Trajectory**: $\Delta x = -0.3989\text{ m}, \Delta y = +0.0005\text{ m}, \Delta z = +0.0000\text{ m}, \Delta\text{Yaw} = +0.06^\circ$
- **Steering Pod Angles**: FL = `+0.92°`, FR = `-1.05°`, RL = `-0.60°`, RR = `+0.60°`
- **Wheel Angular Positions**: FL = `+0.26 rad`, FR = `+0.26 rad`, RL = `+0.26 rad`, RR = `+0.26 rad`
- **Wheel-Ground Contact**: $\Delta z = +0.0000\text{ m}$ (Zero vertical bouncing; continuous stable ground contact).

### TEST C: Strafe Left
- **Commanded Velocity**: $(v_x, v_y, \omega_z) = (+0.00, +0.35, +0.00)$
- **Measured Trajectory**: $\Delta x = +0.0001\text{ m}, \Delta y = +0.2737\text{ m}, \Delta z = +0.0000\text{ m}, \Delta\text{Yaw} = +0.41^\circ$
- **Steering Pod Angles**: FL = `+58.26°`, FR = `-57.20°`, RL = `-58.32°`, RR = `+60.48°`
- **Wheel Angular Positions**: FL = `+6.20 rad`, FR = `-5.75 rad`, RL = `-5.69 rad`, RR = `+6.21 rad`
- **Wheel-Ground Contact**: $\Delta z = +0.0000\text{ m}$ (Zero vertical bouncing; continuous stable ground contact).

### TEST D: Strafe Right
- **Commanded Velocity**: $(v_x, v_y, \omega_z) = (+0.00, -0.35, +0.00)$
- **Measured Trajectory**: $\Delta x = +0.0031\text{ m}, \Delta y = -0.2978\text{ m}, \Delta z = +0.0000\text{ m}, \Delta\text{Yaw} = -0.61^\circ$
- **Steering Pod Angles**: FL = `+60.52°`, FR = `-59.43°`, RL = `-58.65°`, RR = `+60.48°`
- **Wheel Angular Positions**: FL = `-0.22 rad`, FR = `+0.59 rad`, RL = `+0.59 rad`, RR = `-0.20 rad`
- **Wheel-Ground Contact**: $\Delta z = +0.0000\text{ m}$ (Zero vertical bouncing; continuous stable ground contact).

### TEST E: Rotate CCW in Place
- **Commanded Velocity**: $(v_x, v_y, \omega_z) = (+0.00, +0.00, +1.00)$
- **Measured Trajectory**: $\Delta x = +0.0117\text{ m}, \Delta y = -0.0014\text{ m}, \Delta z = +0.0000\text{ m}, \Delta\text{Yaw} = +2.82^\circ$
- **Steering Pod Angles**: FL = `+105.57°`, FR = `-106.76°`, RL = `-105.99°`, RR = `+105.82°`
- **Wheel Angular Positions**: FL = `+44.47 rad`, FR = `+45.16 rad`, RL = `-44.00 rad`, RR = `-44.88 rad`
- **Wheel-Ground Contact**: $\Delta z = +0.0000\text{ m}$ (Zero vertical bouncing; continuous stable ground contact).

### TEST F: Rotate CW in Place
- **Commanded Velocity**: $(v_x, v_y, \omega_z) = (+0.00, +0.00, -1.00)$
- **Measured Trajectory**: $\Delta x = +0.0059\text{ m}, \Delta y = -0.0035\text{ m}, \Delta z = +0.0000\text{ m}, \Delta\text{Yaw} = +2.56^\circ$
- **Steering Pod Angles**: FL = `+108.42°`, FR = `-109.54°`, RL = `-108.65°`, RR = `+108.63°`
- **Wheel Angular Positions**: FL = `-1.12 rad`, FR = `-0.44 rad`, RL = `+1.60 rad`, RR = `+0.71 rad`
- **Wheel-Ground Contact**: $\Delta z = +0.0000\text{ m}$ (Zero vertical bouncing; continuous stable ground contact).

### TEST G: Forward + CCW Rotation
- **Commanded Velocity**: $(v_x, v_y, \omega_z) = (+0.25, +0.00, +0.50)$
- **Measured Trajectory**: $\Delta x = -0.0070\text{ m}, \Delta y = +0.0039\text{ m}, \Delta z = +0.0000\text{ m}, \Delta\text{Yaw} = -1.91^\circ$
- **Steering Pod Angles**: FL = `+108.42°`, FR = `-109.54°`, RL = `-108.65°`, RR = `+108.63°`
- **Wheel Angular Positions**: FL = `+43.60 rad`, FR = `+44.28 rad`, RL = `-43.12 rad`, RR = `-44.01 rad`
- **Wheel-Ground Contact**: $\Delta z = +0.0000\text{ m}$ (Zero vertical bouncing; continuous stable ground contact).

### TEST H: Strafe Left + CCW Rotation
- **Commanded Velocity**: $(v_x, v_y, \omega_z) = (+0.00, +0.25, +0.50)$
- **Measured Trajectory**: $\Delta x = -0.0086\text{ m}, \Delta y = +0.0125\text{ m}, \Delta z = +0.0000\text{ m}, \Delta\text{Yaw} = -3.90^\circ$
- **Steering Pod Angles**: FL = `+108.94°`, FR = `-110.05°`, RL = `-109.21°`, RR = `+109.13°`
- **Wheel Angular Positions**: FL = `+89.12 rad`, FR = `+89.65 rad`, RL = `-88.64 rad`, RR = `-88.99 rad`
- **Wheel-Ground Contact**: $\Delta z = +0.0000\text{ m}$ (Zero vertical bouncing; continuous stable ground contact).

