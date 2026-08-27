# YOLO Target Selection & Autonomous Swerve Navigation Guide

**Robot Platform**: Smart Car 4-Wheel Independent Swerve Drive  
**Environment**: ROS 2 Lyrical / Gazebo Sim 10.4.0 (DART Physics)  
**Vision Model**: Ultralytics YOLOv8-nano (`models/yolov8n.pt`)  

---

## 1. Overview & Capabilities

The Smart Car simulation supports dynamic, real-time YOLOv8 object target selection from the terminal. The robot will search for, identify, track, approach, avoid obstacles around, and arrive at any requested COCO object class.

Supported targets include:
- `chair`
- `person`
- `bottle`
- `cup`
- `car`
- `backpack`
- `all` (detect all classes)

---

## 2. Setting Target Objects Dynamically via Terminal

### A. Select "chair" as the active target
```bash
ros2 topic pub --once /vision/set_target std_msgs/msg/String "{data: 'chair'}"
```

### B. Select "person" as the active target
```bash
ros2 topic pub --once /vision/set_target std_msgs/msg/String "{data: 'person'}"
```

### C. Select "bottle" as the active target
```bash
ros2 topic pub --once /vision/set_target std_msgs/msg/String "{data: 'bottle'}"
```

---

## 3. Autonomous 7-State Navigation State Machine
 
The robot operates under a 7-state autonomous navigation state machine published on `/navigation/state`:

```
           [IDLE]
             │ (nav_mode: follow)
             ▼
          [SEARCH] <────────────────────────────┐
             │ (target detected in frame)        │ (target lost)
             ▼                                   │
     [TARGET_DETECTED]                           │
             │                                   │
             ▼                                   │
        [APPROACH] ──────────────────────────────┤
             │ (obstacle blocks forward path)    │
             ▼                                   │
          [AVOID] (lateral swerve evasion)       │
             │ (obstacle cleared)                │
             ▼                                   │
        [REACQUIRE] (re-center target)           │
             │                                   │
             ▼                                   │
        [APPROACH] ──────────────────────────────┘
             │ (target safe distance reached: w >= 220px)
             ▼
         [ARRIVED] (safe stop at destination)
```


---

## 4. Camera HUD Display Layout

The real-time camera feed published on `/camera/image_annotated` renders an informative HUD banner:

```
┌────────────────────────────────────────────────────────┐
│ Target Mode : CHAIR [yolo]                             │
│ Detection   : chair (85%)                              │
│ Target Center: (320, 240)                              │
│ Nav State   : APPROACH                                 │
│ PID Error   : +0.0 px                                  │
│ Cmd Vel     : vx=+0.35, vy=+0.00                       │
│ Obstacle Det: NO                                       │
│ FPS: 30.0                                              │
└────────────────────────────────────────────────────────┘
```
