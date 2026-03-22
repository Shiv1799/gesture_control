# Gesture-Controlled Robotic Arm

### Real-Time Hand Gesture Control of a Franka Emika Panda using ROS 2, MoveIt Servo & MediaPipe

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Servo-orange)](https://moveit.picknik.ai/humble/)
[![MediaPipe](https://img.shields.io/badge/MediaPipe-Hands-green?logo=google)](https://mediapipe.dev/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)](https://www.python.org/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker)](https://www.docker.com/)
[![License](https://img.shields.io/badge/License-MIT-brightgreen)](LICENSE)

---

> Control a 7-DOF robotic arm with nothing but your hand and a webcam. No gloves, no depth camera, no special hardware — just a standard 2D webcam and computer vision.

---

## Demo

<!-- Replace with your actual GIF/video after recording -->
<!-- ![Demo GIF](assets/robotic_arm_cv.mp4) -->

| Hand Open — Arm Tracks | Pinch — Gripper Closes | Fist — Emergency Stop |
|:-----------------------:|:----------------------:|:---------------------:|
| Move hand to steer arm  | Pinch thumb + index    | Close fist to freeze  |

```
+------------------+     ROS 2 Topics      +------------------+     MoveIt Servo     +------------------+
|                  |  /gesture/target_pose  |                  |  TwistStamped @30Hz  |                  |
|  Webcam + Hand   | ───────────────────▶   |  Gesture Bridge  | ───────────────────▶ |   Franka Panda   |
|  Tracking Node   |  /gesture/gripper_cmd  |  (TF2 Feedback)  |  IK + Collision Chk  |   7-DOF Arm      |
|                  |  /gesture/estop        |                  |  Singularity Avoid.  |                  |
+------------------+                        +------------------+                       +------------------+
     MediaPipe                                 Position-Error                            Real-Time
     21 Landmarks                              Control Loop                              Joint Trajectories
```

---

## Key Features

- **Real-time position tracking** — TF2 feedback loop computes `velocity = gain × (target − current_EE)`, so the arm tracks your hand position with zero drift
- **MoveIt Servo integration** — Cartesian velocity control at 30 Hz with built-in IK, collision checking, joint limits, and singularity avoidance
- **3 intuitive gestures** — Open hand (move), pinch (gripper), fist (emergency stop)
- **Camera feed in RViz** — Annotated webcam stream published as `sensor_msgs/Image` on `/gesture/camera_feed`, viewable directly in RViz alongside the 3D robot
- **Single launch file** — One command brings up MoveIt, Servo, gesture tracking, and the bridge with proper startup sequencing
- **Fully containerized** — Docker image based on `moveit/moveit2:humble-release`, works on any Linux machine with a webcam

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          System Architecture                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────┐                                                          │
│   │   Webcam      │                                                         │
│   │  /dev/video0  │                                                         │
│   └──────┬───────┘                                                          │
│          │ BGR frames @ 30fps                                               │
│          ▼                                                                  │
│   ┌──────────────────────────────────────────┐                              │
│   │  gesture_tracker (hand_gesture_node)     │                              │
│   │  ┌────────────────────────────────────┐  │                              │
│   │  │ MediaPipe Hands (21 2D landmarks)  │  │                              │
│   │  └────────────┬───────────────────────┘  │                              │
│   │               │                          │                              │
│   │  ┌────────────▼───────────────────────┐  │     ┌──────────────────┐     │
│   │  │ Gesture Interpretation             │  │     │ /gesture/        │     │
│   │  │  • Wrist XY → Robot YZ position    │──┼────▶│  camera_feed     │     │
│   │  │  • Pinch detect → Gripper cmd      │  │     │  (Image for RViz)│     │
│   │  │  • Fist detect  → Emergency stop   │  │     └──────────────────┘     │
│   │  │  • EMA smoothing (α = 0.4)         │  │                              │
│   │  └────────────────────────────────────┘  │                              │
│   └──────┬───────────┬──────────┬────────────┘                              │
│          │           │          │                                            │
│     target_pose  gripper_cmd  estop                                         │
│          │           │          │                                            │
│          ▼           ▼          ▼                                            │
│   ┌──────────────────────────────────────────┐     ┌──────────────────┐     │
│   │  gesture_bridge (gesture_control)        │     │  TF2             │     │
│   │  ┌────────────────────────────────────┐  │◀────│  panda_link0 →   │     │
│   │  │ Position-Error Control Loop        │  │     │  panda_link8     │     │
│   │  │                                    │  │     │  (EE position)   │     │
│   │  │  error = target − current_EE       │  │     └──────────────────┘     │
│   │  │  velocity = gain × error           │  │                              │
│   │  │  clamp(velocity, ±0.5 m/s)         │  │                              │
│   │  └────────────┬───────────────────────┘  │                              │
│   └───────────────┼──────────────────────────┘                              │
│                   │ TwistStamped @ 30Hz                                     │
│                   ▼                                                         │
│   ┌──────────────────────────────────────────┐                              │
│   │  MoveIt Servo (servo_node)               │                              │
│   │  • Real-time inverse kinematics          │                              │
│   │  • Collision checking @ 10Hz             │                              │
│   │  • Singularity avoidance                 │                              │
│   │  • Joint limit enforcement               │                              │
│   └───────────────┬──────────────────────────┘                              │
│                   │ JointTrajectory                                         │
│                   ▼                                                         │
│   ┌──────────────────────────────────────────┐                              │
│   │  ros2_control (mock hardware)            │                              │
│   │  • panda_arm_controller (JointTraj)      │                              │
│   │  • panda_hand_controller (Gripper)       │                              │
│   └──────────────────────────────────────────┘                              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Coordinate Mapping

A key design decision: mapping a **2D camera** to a **3D robot workspace**.

| Camera Axis | Robot Axis | Range | Description |
|:-----------:|:----------:|:-----:|:------------|
| X (left/right) | **Y** (sideways) | -0.3 → 0.3 m | Hand left = robot left |
| Y (up/down) | **Z** (height) | 0.15 → 0.65 m | Hand up = arm up |
| — | **X** (forward) | Fixed 0.4 m | No depth from 2D camera |

```
Camera View (mirrored)          Panda Base Frame (panda_link0)
┌─────────────────┐                    Z ▲ (height)
│  ↕ Y (up/down)  │                      │
│  ↔ X (left/rt)  │       ──────▶        │    Y
│                  │       mapping        └──────▶ (sideways)
│   [hand here]   │                      X goes into screen (fixed)
└─────────────────┘
```

---

## Gesture Reference

| Gesture | Detection Method | Threshold | Robot Action |
|:--------|:-----------------|:---------:|:-------------|
| **Open Hand** | Default state | — | Arm tracks hand position (Y, Z) |
| **Pinch** | Thumb tip ↔ Index tip 2D distance | < 0.07 | Close gripper (0.0 m) |
| **Release** | Thumb tip ↔ Index tip 2D distance | ≥ 0.07 | Open gripper (0.04 m) |
| **Fist** | All 5 fingertips within palm radius | < 0.10 | **Emergency stop** — zero velocity |

---

## Tech Stack

| Component | Technology | Role |
|:----------|:-----------|:-----|
| Robot Simulation | **Franka Emika Panda** (7-DOF) via `moveit_resources` | Robotic arm with gripper |
| Middleware | **ROS 2 Humble** | Communication, TF2, launch system |
| Motion Planning | **MoveIt 2 Servo** | Real-time IK, collision avoidance, singularity handling |
| Control | **ros2_control** (mock hardware) | Joint trajectory execution |
| Hand Tracking | **MediaPipe Hands** | 21 2D landmarks per hand, ~30 fps |
| Computer Vision | **OpenCV** + **cv_bridge** | Frame capture, annotation, ROS Image publishing |
| Visualization | **RViz2** | 3D robot + camera feed in one window |
| Containerization | **Docker** (`moveit/moveit2:humble-release`) | Reproducible environment |

---

## Package Structure

```
gesture_arm_ws/
├── src/
│   ├── hand_gesture_node/          # Perception layer
│   │   ├── hand_gesture_node/
│   │   │   └── gesture_tracker.py  # MediaPipe tracking + Image publisher
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── gesture_control/            # Control layer
│   │   ├── gesture_control/
│   │   │   └── gesture_bridge.py   # TF2 feedback + Servo velocity control
│   │   ├── config/
│   │   │   └── gesture_params.yaml # Bridge tuning parameters
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── panda_gesture_bringup/      # Orchestration layer
│       ├── launch/
│       │   ├── gesture_panda_all.launch.py   # ★ Unified launch (recommended)
│       │   ├── panda_gazebo_moveit.launch.py # MoveIt + Servo only
│       │   └── gesture_control.launch.py     # Gesture nodes only
│       ├── config/
│       │   ├── panda_servo_config.yaml       # MoveIt Servo tuning
│       │   └── workspace_bounds.yaml         # Workspace mapping config
│       ├── package.xml
│       └── setup.py
│
├── Dockerfile
├── run.bash
└── README.md
```

---

## Quick Start

### Prerequisites

- Linux (Ubuntu 22.04 recommended)
- Docker
- USB webcam

### 1. Clone & Build Docker Image

```bash
git clone https://github.com/Shiv1799/gesture_control.git
cd gesture_control/gesture_arm_ws
docker build -t panda-gesture:latest .
```

### 2. Launch Container

```bash
./run.bash
```

> This enables X11 forwarding for GUI (RViz + camera), passes through `/dev/video0`, and runs with `--net=host`.

### 3. Build Workspace (inside container)

```bash
cd /gesture_arm_ws
source /opt/ros/humble/setup.bash
pip install mediapipe opencv-python
colcon build --symlink-install
source install/setup.bash
```

### 4. Run Everything

```bash
ros2 launch panda_gesture_bringup gesture_panda_all.launch.py
```

That's it. RViz opens with the Panda robot. After ~5 seconds the gesture tracker starts. Add an **Image** display in RViz subscribed to `/gesture/camera_feed` to see your hand tracking alongside the robot.

### Launch Arguments

```bash
ros2 launch panda_gesture_bringup gesture_panda_all.launch.py \
  camera_id:=0 \
  velocity_gain:=3.0 \
  smoothing_factor:=0.4 \
  use_rviz:=true
```

| Argument | Default | Description |
|:---------|:-------:|:------------|
| `camera_id` | `0` | Webcam device index |
| `velocity_gain` | `3.0` | Arm responsiveness (higher = faster) |
| `smoothing_factor` | `0.4` | Hand position smoothing (higher = smoother) |
| `use_rviz` | `true` | Launch RViz visualization |

---

## ROS 2 Topics

| Topic | Type | Hz | Description |
|:------|:-----|:--:|:------------|
| `/gesture/target_pose` | `geometry_msgs/Pose` | 30 | Target EE position from hand |
| `/gesture/gripper_command` | `std_msgs/Bool` | 30 | `True` = close, `False` = open |
| `/gesture/emergency_stop` | `std_msgs/Bool` | 30 | `True` when fist detected |
| `/gesture/camera_feed` | `sensor_msgs/Image` | 30 | Annotated webcam frame for RViz |
| `/servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` | 30 | Velocity commands to Servo |
| `/servo_node/status` | `std_msgs/Int8` | 100 | Servo status (1=OK, -1=collision) |

---

## Configuration & Tuning

<details>
<summary><b>Gesture Detection Thresholds</b> — <code>workspace_bounds.yaml</code></summary>

```yaml
gesture_tracker:
  ros__parameters:
    pinch_threshold: 0.07    # Lower = harder to trigger pinch
    fist_threshold: 0.10     # Lower = harder to trigger e-stop
    smoothing_factor: 0.4    # 0.0 = raw, 1.0 = max smoothing
```
</details>

<details>
<summary><b>Arm Speed & Responsiveness</b> — <code>gesture_params.yaml</code></summary>

```yaml
gesture_bridge:
  ros__parameters:
    velocity_gain: 3.0       # P-controller gain (higher = faster tracking)
    max_linear_speed: 0.5    # Hard limit (m/s)
    position_tolerance: 0.005 # Stop within 5mm of target
    deadzone: 0.01           # Ignore sub-1cm errors
```
</details>

<details>
<summary><b>MoveIt Servo Safety</b> — <code>panda_servo_config.yaml</code></summary>

```yaml
servo_node:
  ros__parameters:
    scale:
      linear: 0.4            # Servo velocity scale (m/s)
      rotational: 0.8        # Rotation scale (rad/s)
    check_collisions: true
    collision_check_rate: 10.0
    joint_limit_margin: 0.1   # Radians from joint limits
    lower_singularity_threshold: 17.0
    hard_stop_singularity_threshold: 30.0
```
</details>

---

## Design Decisions

| Decision | Rationale |
|:---------|:----------|
| **MoveIt Servo** over MoveGroup plan+execute | Gesture control needs continuous real-time response (~30 Hz), not discrete trajectory planning |
| **TF2 position-error control** over open-loop velocity | Closed-loop `v = K(target − current)` eliminates drift and ensures the arm converges to the hand position |
| **2D-only tracking** (no depth) | Standard webcams are ubiquitous; MediaPipe's Z estimate from monocular is unreliable. Fixed forward reach is a practical tradeoff |
| **cv_bridge Image publishing** over OpenCV windows | Keeps visualization in RViz (single window), works over SSH/Docker, and follows ROS conventions |
| **TimerAction delayed startup** | MoveIt + Servo need ~5s to initialize; delayed gesture node launch prevents race conditions |
| **EMA smoothing** on hand position | Eliminates jitter from MediaPipe landmark noise without adding latency |

---

## Troubleshooting

<details>
<summary><b>Arm doesn't move</b></summary>

1. Check Servo started: `ros2 topic echo /servo_node/status` (should show `1`)
2. Check twist commands: `ros2 topic echo /servo_node/delta_twist_cmds`
3. Check TF is available: `ros2 run tf2_ros tf2_echo panda_link0 panda_link8`
4. Ensure gesture_bridge logs `MoveIt Servo started successfully`
</details>

<details>
<summary><b>Webcam not found</b></summary>

1. Check devices: `ls /dev/video*`
2. Try different camera_id: `camera_id:=1`
3. Ensure Docker has `--privileged` and passes `/dev/video0`
</details>

<details>
<summary><b>Arm moves erratically</b></summary>

1. Decrease `velocity_gain` (try `1.5`)
2. Increase `smoothing_factor` (try `0.6`)
3. Decrease `max_linear_speed` (try `0.25`)
</details>

<details>
<summary><b>Camera feed not showing in RViz</b></summary>

1. In RViz: **Add** → **By topic** → `/gesture/camera_feed` → **Image**
2. Check topic: `ros2 topic hz /gesture/camera_feed` (should be ~30 Hz)
</details>

---

## Future Improvements

- [ ] Depth estimation using monocular depth models (MiDaS) for full 3D control
- [ ] Multi-hand support — one hand for position, other for gripper/rotation
- [ ] Wrist rotation tracking → end-effector orientation control
- [ ] Integration with real Franka hardware via `franka_ros2`
- [ ] Voice command overlay for mode switching
- [ ] Recording and replay of gesture trajectories

---

## References

- [MoveIt Servo — Real-Time Arm Servoing](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [MediaPipe Hands](https://mediapipe.dev/solutions/hands)
- [Franka Emika Panda — MoveIt Config](https://github.com/moveit/moveit_resources/tree/humble/panda_moveit_config)
- [ros2_control — Hardware Abstraction](https://control.ros.org/humble/)

---

<p align="center">
  Built with ROS 2 Humble • MoveIt 2 • MediaPipe • OpenCV
</p>

