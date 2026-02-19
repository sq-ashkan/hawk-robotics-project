# TurtleBot3 Line Follower

Autonomous line-following robot using ROS2 Jazzy with both Gazebo Harmonic simulation and real TurtleBot3 Waffle hardware.

**Course:** Autonomous Systems
**University:** HAWK Hochschule
**Developer:** Ashkan Sadri Ghamshi | Mohamed Wajih | Edem Mejri
**Supervisor:** Prof. Thomas Linkugel
**Date:** February 2026

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage - Simulation](#usage---simulation)
- [Usage - Real Robot](#usage---real-robot)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Technical Details](#technical-details)
- [License](#license)

---

## Overview

This project implements an autonomous line-following robot using the TurtleBot3 Waffle platform. The system supports two operating modes:

1. **Simulation Mode** - Gazebo Harmonic with bird's-eye view transformation and PID control
2. **Real Robot Mode** - Physical TurtleBot3 Waffle with binary threshold detection and continuous proportional control

### Simulation Pipeline
- Camera calibration and undistortion
- Bird's-eye view (homography) transformation
- White line detection using thresholding
- Cross-track error calculation
- PID-based velocity control with curvature-aware speed adjustment

### Real Robot Pipeline
- IMX708 camera (800x600), mounted upside-down on TurtleBot3 Waffle
- Software rotation 180 degrees to correct orientation
- Binary threshold detection on cropped floor region
- Multi-height line scanning at 6 heights for robust detection
- Continuous proportional controller with curve-aware speed adjustment

---

## Features

- Real-time lane detection using OpenCV
- Bird's-eye view transformation for simulation
- PID controller with anti-windup and dead zone (simulation)
- Continuous proportional controller (real robot)
- Multi-height line detection algorithm with 6 scan lines
- Curvature-aware speed control (slows in curves)
- LiDAR point overlay on bird's-eye view
- Multiple debug image topics for visualization
- Support for both left and right curve tracks
- Configurable parameters via YAML files

---

## System Architecture

### Simulation Mode

```
+-------------------------------------------------------------+
|                    Line Follower Node                        |
+-------------------------------------------------------------+
|                                                              |
|  +----------+    +----------+    +----------+               |
|  | Camera   |--->| Calibra- |--->| Bird's-  |               |
|  | Input    |    | tion     |    | Eye View |               |
|  +----------+    +----------+    +----------+               |
|                                        |                     |
|                                        v                     |
|  +----------+    +----------+    +----------+               |
|  | Velocity |<---| PID      |<---| Line     |               |
|  | Command  |    | Control  |    | Detector |               |
|  +----------+    +----------+    +----------+               |
|                                                              |
+-------------------------------------------------------------+
```

### Real Robot Mode

```
+-------------------------------------------------------------+
|               Real Robot Line Follower Node                  |
+-------------------------------------------------------------+
|                                                              |
|  +----------+    +----------+    +----------+               |
|  | Camera   |--->| Rotate   |--->| Binary   |               |
|  | (IMX708) |    | 180 deg  |    | Thresh.  |               |
|  +----------+    +----------+    +----------+               |
|                                        |                     |
|                                        v                     |
|  +----------+    +----------+    +----------+               |
|  | Velocity |<---| Proport. |<---| Multi-   |               |
|  | Command  |    | Control  |    | Height   |               |
|  +----------+    +----------+    +----------+               |
|                                                              |
+-------------------------------------------------------------+
```

### Modules

| Module | Mode | Description |
|--------|------|-------------|
| `line_follower_node.py` | Simulation | Main ROS2 node, orchestrates simulation pipeline |
| `calibration.py` | Simulation | Camera calibration and undistortion |
| `birdseye.py` | Simulation | Homography transformation |
| `line_detector.py` | Simulation | Lane detection and center calculation |
| `controller.py` | Simulation | PID controller with speed adjustment |
| `lidar_transform.py` | Simulation | LiDAR to image coordinate transform |
| `real_robot_line_follower_node.py` | Real Robot | Main ROS2 node for real hardware |
| `real_line_detector.py` | Real Robot | Binary threshold multi-height line detection |
| `real_robot_controller.py` | Real Robot | Continuous proportional controller |

---

## Prerequisites

### System Requirements

- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- Gazebo Harmonic (gz-harmonic) - for simulation
- Python 3.12+

### ROS2 Packages

```bash
sudo apt install ros-jazzy-turtlebot3-gazebo
sudo apt install ros-jazzy-turtlebot3-description
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-rqt-image-view
```

### Python Dependencies

```bash
pip install opencv-python numpy pyyaml
```

---

## Installation

1. **Clone the repository**

```bash
git clone <repository-url> hawk-robotics-project
cd hawk-robotics-project
```

2. **Build the workspace**

```bash
colcon build --symlink-install
```

3. **Source the workspace**

```bash
source install/setup.bash
```

---

## Usage - Simulation

### Step 1: Launch Gazebo Simulation

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2: Add Track to Simulation

Wait for Gazebo to fully load, then run:

```bash
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "<full-path-to>/hawk-robotics-project/src/line_follower_pkg/models/track_ground/model.sdf", name: "track_ground", pose: {position: {x: 5, y: -5, z: 0.005}}'
```

### Step 3: Position the Robot

**Track 1 (Left Curve):**
```bash
gz service -s /world/default/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'name: "waffle", position: {x: 5.0, y: -9.7, z: 0.1}, orientation: {x: 0, y: 0, z: 0.707, w: 0.707}'
```

**Track 2 (Right Curve):**
```bash
gz service -s /world/default/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'name: "waffle", position: {x: 6.5, y: -9.7, z: 0.1}, orientation: {x: 0, y: 0, z: 0.707, w: 0.707}'
```

### Step 4: Run Line Follower

```bash
source install/setup.bash
ros2 run line_follower_pkg line_follower_node
```

### Step 5: View Camera Output (Optional)

```bash
ros2 run rqt_image_view rqt_image_view
```

Available topics:
- `/camera/image_raw` - Raw camera image
- `/camera/rectified_image` - Undistorted image
- `/camera/birdseye_image` - Bird's-eye view
- `/camera/birdseye_with_lidar` - Bird's-eye view with LiDAR overlay
- `/camera/line_detection` - Image with detected lines drawn

---

## Usage - Real Robot

### Prerequisites

- TurtleBot3 Waffle with Raspberry Pi 5
- IMX708 camera module (mounted upside-down)
- SSH access to the robot (`ssh -l turtlebot amrl-turtlebot2`, password: `turtlebot`)
- Workstation and robot on the same network (AMRL lab network)

### Overview: 3 Terminals Required

| Terminal | Location | Purpose |
|----------|----------|---------|
| SSH Terminal 1 | Robot | TurtleBot3 bringup (motors, sensors) |
| SSH Terminal 2 | Robot | Camera node |
| Local Terminal | Workstation (PC) | Line follower algorithm |

### Step 1: SSH Terminal 1 - Robot Bringup

```bash
ssh -l turtlebot amrl-turtlebot2
# Password: turtlebot

# On robot:
ros2 launch turtlebot3_bringup robot.launch.py
```

This starts the robot's motor drivers, IMU, and LiDAR.

### Step 2: SSH Terminal 2 - Start Camera

```bash
ssh -l turtlebot amrl-turtlebot2
# Password: turtlebot

# On robot:
ros2 run camera_ros camera_node
```

This publishes the IMX708 camera feed to `/camera/image_raw`.

### Step 3: Local Terminal - Run Line Follower

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/hawk-robotics-project
source install/setup.bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 run line_follower_pkg real_robot_line_follower
```

The robot will start moving and following the white lines on the floor.

### Step 4: Monitor Detection (Optional)

In a separate terminal on the workstation:

```bash
ros2 run rqt_image_view rqt_image_view
# Select topic: /camera/line_detection
```

Debug images with scan lines and detected points are automatically saved to `~/Desktop/` for analysis.

---

## Configuration

### Simulation - PID Controller Parameters

Edit `controller.py` to adjust:

```python
# Steering PID gains
kp = 2.0    # Proportional gain
ki = 0.02   # Integral gain
kd = 0.3    # Derivative gain

# Speed settings (m/s)
base_speed = 0.12
min_speed = 0.06
max_speed = 0.15
```

### Simulation - Homography Calibration

Run the calibration tool to adjust bird's-eye view transformation:

```bash
ros2 run line_follower_pkg calibrate_homography
```

Controls:
- **Click and drag** - Move corner points
- **R** - Reset to defaults
- **P** - Preview transformation
- **S** - Save configuration
- **Q** - Quit

Configuration is saved to `config/homography.yaml`.

### Real Robot - Controller Parameters

Edit `config/real_robot_params.yaml`:

```yaml
control:
  forward_speed: 0.02       # m/s constant forward speed
  angular_gain: 0.2         # steering gain: angular = -offset * gain
  max_angular: 0.3          # max angular velocity (rad/s)
```

### Line Detection Threshold

**Simulation:** Edit `line_detector.py`:
```python
self.white_threshold = 110  # Pixel brightness threshold
```

**Real Robot:** Edit `real_line_detector.py`:
```python
self.threshold = 200  # Binary threshold for white line detection
```

---

## Project Structure

```
hawk-robotics-project/
├── src/
│   ├── line_follower_pkg/
│   │   ├── config/
│   │   │   ├── homography.yaml
│   │   │   ├── real_robot_params.yaml
│   │   │   └── real_robot_homography.yaml
│   │   ├── launch/
│   │   │   ├── line_follower.launch.py
│   │   │   ├── line_track.launch.py
│   │   │   └── real_robot_line_follower.launch.py
│   │   ├── line_follower_pkg/
│   │   │   ├── __init__.py
│   │   │   ├── line_follower_node.py
│   │   │   ├── calibration.py
│   │   │   ├── birdseye.py
│   │   │   ├── line_detector.py
│   │   │   ├── controller.py
│   │   │   ├── lidar_transform.py
│   │   │   ├── calibrate_homography.py
│   │   │   ├── calibrate_camera.py
│   │   │   └── real_robot/
│   │   │       ├── __init__.py
│   │   │       ├── real_robot_line_follower_node.py
│   │   │       ├── real_line_detector.py
│   │   │       └── real_robot_controller.py
│   │   ├── models/
│   │   │   ├── track_ground/
│   │   │   │   └── model.sdf
│   │   │   ├── curved_track/
│   │   │   │   └── model.sdf
│   │   │   └── white_line/
│   │   │       └── model.sdf
│   │   ├── scripts/
│   │   │   ├── generate_chessboard.py
│   │   │   ├── generate_curved_track.py
│   │   │   └── generate_track_texture.py
│   │   ├── worlds/
│   │   │   ├── line_track.sdf
│   │   │   ├── simple_lines.sdf
│   │   │   └── textures/
│   │   │       └── curved_track.png
│   │   ├── package.xml
│   │   └── setup.py
└── README.md
```

---

## Technical Details

### Bird's-Eye View Transformation (Simulation)

The homography transformation maps a trapezoidal region in the camera image to a rectangular bird's-eye view:

```
Camera View              Bird's-Eye View
    ____                  ________
   /    \                |        |
  /      \      H        |        |
 /        \   ====>      |        |
/          \             |________|
```

The transformation matrix H is calculated using `cv2.getPerspectiveTransform()` with 4 corresponding point pairs.

### Line Detection - Simulation

1. Convert image to grayscale
2. Apply Gaussian blur (5x5 kernel)
3. Threshold to isolate white lines (threshold = 110)
4. Scan rows from bottom to middle
5. Find left and right line positions using median filtering
6. Calculate lane center as midpoint between lines
7. Apply temporal smoothing (10-frame history)

### Line Detection - Real Robot (Multi-Height Scan)

The real robot uses a different detection approach optimized for the physical environment:

1. Crop bottom 50% of the rotated camera image (floor region)
2. Resize to 640x480 and convert to grayscale
3. Apply binary threshold (200) to isolate white lines on dark floor
4. Morphological erosion (3x3) and dilation (5x5) for noise removal
5. Connected component filtering to remove small artifacts
6. Scan 6 horizontal lines at heights 25%, 35%, 45%, 55%, 65%, 75% from top
7. Cluster white pixels to find left and right line positions at each height
8. Calculate lane center from paired detections (both lines visible at same height)
9. Prioritize nearest paired center for steering, with 20% weight on farthest for curve anticipation
10. Apply temporal smoothing (5-frame moving average) to reduce jitter
11. Single-line fallback: when only one boundary is visible, estimate lane center using last known lane width

### Continuous Proportional Controller (Real Robot)

The real robot uses continuous proportional control:

- Always moves forward at constant low speed (0.02 m/s)
- Steering: `angular_velocity = -offset * angular_gain`
- Slows down by 50% when offset exceeds curve threshold
- Stops after 20 consecutive frames without line detection

### PID Controller (Simulation)

The steering uses a PID controller with:
- Anti-windup limits on integral term
- Dead zone (5%) to prevent oscillation near center
- Output clamping to max angular velocity

Speed is adjusted based on detected curvature:
- Sharp curve (< 300px): 40% base speed
- Medium curve (< 800px): 60% base speed
- Gentle curve (< 2000px): 80% base speed
- Straight: 100% base speed

### ROS2 Topics

**Simulation:**

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Raw camera input |
| `/camera/rectified_image` | sensor_msgs/Image | Undistorted image |
| `/camera/birdseye_image` | sensor_msgs/Image | Bird's-eye view |
| `/camera/birdseye_with_lidar` | sensor_msgs/Image | Bird's-eye with LiDAR |
| `/camera/line_detection` | sensor_msgs/Image | Debug visualization |
| `/cmd_vel` | geometry_msgs/TwistStamped | Velocity commands |
| `/scan` | sensor_msgs/LaserScan | LiDAR input |

**Real Robot:**

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Camera input (IMX708) |
| `/camera/line_detection` | sensor_msgs/Image | Debug visualization |
| `/cmd_vel` | geometry_msgs/TwistStamped | Velocity commands |

---

## License

This project is developed for educational purposes as part of the Autonomous Systems course at HAWK Hochschule.

---

## Acknowledgments

- TurtleBot3 by ROBOTIS
- ROS2 Jazzy by Open Robotics
- Gazebo by Open Robotics
- OpenCV by Intel and Willow Garage
