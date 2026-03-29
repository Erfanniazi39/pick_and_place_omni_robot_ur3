# Pick and Place with Color Matching

A ROS2 pick-and-place manipulation system that combines mobile base navigation, object detection, and robotic arm control to autonomously pick a small cube and place it next to a matching-colored larger cube.

## Overview

The Color Match system orchestrates a coordinated workflow:
1. **Spawn** - Place cubes in the Gazebo simulation
2. **Detect** - Identify the small cube's color via camera vision
3. **Pick** - Grasp and lift the small cube using the UR3 arm
4. **Scan & Match** - Rotate the base to scan larger cubes, detect their colors, and find a color match
5. **Place** - Navigate the base to the matched target cube and place the small cube beside it

This is implemented as a sequential multi-node pipeline with explicit message-based handoffs, ensuring deterministic operation and tight control over timing and coordination.

## Architecture

The system consists of four coordinated ROS2 nodes running in sequence:

### Node Pipeline

```
cube_spawner (t=0)
      ↓
detect_small_cubes (t=1×wait_sec)
      ↓
pick_middle_cube (t=2×wait_sec)
      ↓
rotate_scan_color (t=3×wait_sec)
```

## System Requirements

- **OS**: Ubuntu 22.04 or later
- **ROS2**: Jazzy (or Humble with Python 3.10+)
- **Python**: 3.10+
- **Gazebo**: Gazebo Sim (garden or later)
- **Dependencies**:
  - `ros2_control`
  - `moveit2`
  - OpenCV (via `opencv-python`)
  - `cv_bridge`

## Installation

1. **Clone into your ROS2 workspace**:
   ```bash
   cd ~/ros2_ws/src
   git clone <repo-url> pick_and_place
   cd ~/ros2_ws
   ```

2. **Install dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select pick_and_place
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Running the System

### Basic Launch

```bash
ros2 launch pick_and_place color_match.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch pick_and_place color_match.launch.py wait_sec:=4.0
```

**Parameters**:
- `wait_sec` (default: 3.0) - Delay in seconds between node startups

## Node Descriptions

### 1. cube_spawner
**Purpose**: Spawn test objects in Gazebo simulation

**Behavior**:
- Spawns 1 small cube (random color: red/green/blue) at center of table
- Spawns 3 large cubes at fixed positions with randomized colors
- Exits after spawning

**Configuration**:
- Small cube position: `(0.7, 0.0, 0.215)` m in base_footprint frame
- Large cube positions:
  - Left: `(0.0, 1.5, ...)` label="left"
  - Behind: `(-1.5, 0.0, ...)` label="behind"
  - Right: `(0.0, -1.5, ...)` label="right"

### 2. detect_small_cubes
**Purpose**: Detect the small cube's color via camera vision

**Topics**:
- **Publishes**: `/color_match/small_cube_color` (String, latched)
  - Data: "red" | "green" | "blue"

**Behavior**:
- Subscribes to `/camera/image_raw`
- Converts RGB image to HSV color space
- Detects dominant color using contour area
- Publishes color once and exits

**Tuning Parameters**:
- `min_area` (default: 120) - Minimum contour area to qualify as detection
- `report_every_n` (default: 10) - Log frequency while waiting for clear detection

**HSV Color Ranges**:
- **Red**: Hue (0-10, 170-180), Sat/Val (80-255)
- **Green**: Hue (40-85), Sat/Val (80-255)
- **Blue**: Hue (95-130), Sat/Val (80-255)

### 3. pick_middle_cube
**Purpose**: Pick the small cube and place it next to a matched-color large cube

**Topics**:
- **Subscribes**:
  - `/color_match/small_cube_color` (String, latched) - Input color
  - `/color_match/target_big_cube` (String, latched) - Target location from scanner
  - `/odom` (Odometry) - Base odometry for navigation
  - `/camera/image_raw` (Image) - Camera feed

- **Publishes**:
  - `/color_match/picked_cube_color` (String, latched) - Confirms color after grasping
  - `/color_match/start_scan` (Bool, latched) - Trigger for rotate_scan_color
  - `/cmd_vel` (Twist) - Base velocity commands

**Behavior**:
1. Wait for `/color_match/small_cube_color` detection (timeout: 30s)
2. Initialize arm to home pose, move to small cube pickup location `(0.7, 0.0, 0.28)`
3. Execute grasp sequence:
   - Approach (z-offset: 0.14 m above table)
   - Grasp (z: 0.015 m)
   - Lift (z: 0.20 m)
4. Publish picked color and scan trigger
5. Wait for target big cube assignment (timeout: 180s)
6. Navigate to target location and ungrasp

**Navigation Parameters** (tunable):
- `nav_max_lin_speed` (default: 0.20) m/s - Maximum forward speed
- `nav_min_lin_speed` (default: 0.04) m/s - Minimum forward speed
- `nav_max_ang_speed` (default: 0.30) rad/s - Maximum rotation speed
- `nav_heading_kp` (default: 1.3) - Proportional gain for yaw control
- `nav_lin_tolerance` (default: 0.08) m - Position tolerance (stop distance)
- `nav_ang_tolerance` (default: 0.05) rad - Heading tolerance
- `nav_turn_only_threshold` (default: 0.18) rad - Heading error threshold to turn-only

**Navigation Strategy**:
- **Heading-correction mode** (when `|heading_error| > turn_only_threshold`): Rotate in place only
- **Forward-drive mode** (otherwise): Forward movement with simultaneous heading correction
- **Speed ramping**: Linear speed scales with distance (`0.55 × distance`), reduced by 0.35× when heading error remains large
- **Proportional feedback**: Angular velocity = `kp × heading_error` (clamped to ±max_angular_speed)

### 4. rotate_scan_color
**Purpose**: Rotate base 90° increments while scanning large cubes until color match found

**Topics**:
- **Subscribes**:
  - `/color_match/picked_cube_color` (String, latched) - Color to match
  - `/color_match/start_scan` (Bool, latched) - Gate to begin scanning
  - `/odom` (Odometry) - Base odometry for closed-loop rotation
  - `/camera/image_raw` (Image) - Camera feed for large cube detection

- **Publishes**:
  - `/color_match/target_big_cube` (String, latched) - Match result ("left"/"behind"/"right")
  - `/cmd_vel` (Twist) - Base rotation commands

**Behavior**:
1. Wait for both `/color_match/start_scan=True` AND `/color_match/picked_cube_color` (indefinitely)
2. Scan loop (repeating):
   - Rotate base left 90° (closed-loop proportional yaw control)
   - Detect large cube color at current orientation
   - If color matches `picked_color`: publish target label and exit
   - Else: continue to next scanning position (left → behind → right → left ...)

**Rotation Parameters** (tunable):
- `angular_speed` (default: 0.35) rad/s - Maximum rotation speed
- `yaw_kp` (default: 1.2) - Proportional gain for yaw error
- `yaw_tolerance` (default: 0.04) rad - Rotation convergence tolerance (~2.3°)
- `min_angular_speed` (default: 0.08) rad/s - Minimum speed floor to prevent stalling
- `rotate_timeout_sec` (default: 15.0) s - Timeout for main rotation phase
- `fine_correct_timeout_sec` (default: 1.5) s - Timeout for fine-correction phase at reduced speed

**Rotation Strategy**:
- **Main rotation phase**: Proportional yaw control with slowdown near target
- **Fine-correction phase** (1.5s): Reduced speed limits (±0.15 rad/s max, ±0.05 min) for final alignment
- **Fallback**: Open-loop timed rotation if odometry unavailable

## Configuration

### ROS2 Parameter Overrides

All tuning parameters can be overridden at launch time:

```bash
ros2 launch pick_and_place color_match.launch.py \
  wait_sec:=4.0 \
  angular_speed:=0.28 \
  nav_heading_kp:=1.5 \
  nav_lin_tolerance:=0.10
```

### Adjusting Color Detection

Edit HSV thresholds in:
- `detect_small_cubes.py` → `_build_masks()`
- `rotate_scan_color.py` → `_detect_large_cube_color()`

Example for brighter lighting:
```python
red1 = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))  # Higher Sat/Val
red2 = cv2.inRange(hsv, (170, 100, 80), (180, 255, 255))
```

## Message Topics

### Published Topics

- **`/color_match/small_cube_color`** (std_msgs/String, latched)
  - Detected color of small cube
  - Values: "red", "green", "blue"
  - Publisher: `detect_small_cubes`

- **`/color_match/picked_cube_color`** (std_msgs/String, latched)
  - Confirmed color after grasping small cube
  - Publisher: `pick_middle_cube` (after successful grasp)

- **`/color_match/start_scan`** (std_msgs/Bool, latched)
  - Gate signal to begin rotation/scanning
  - Publisher: `pick_middle_cube` (after grasping)

- **`/color_match/target_big_cube`** (std_msgs/String, latched)
  - Matching large cube target location
  - Values: "left", "behind", "right"
  - Publisher: `rotate_scan_color` (on color match)

### Subscribed Topics

- **`/camera/image_raw`** (sensor_msgs/Image)
  - Camera feed (typically from simulated RGB camera)
  - Required by: `detect_small_cubes`, `rotate_scan_color`

- **`/odom`** (nav_msgs/Odometry)
  - Base odometry (position, velocity, heading)
  - Required by: `pick_middle_cube`, `rotate_scan_color`

- **`/cmd_vel`** (geometry_msgs/Twist)
  - Base velocity command (published by navigation nodes)

## Troubleshooting

### Nodes Timeout Waiting for Messages

**Symptom**: "Still waiting for `/color_match/start_scan` and `/color_match/picked_cube_color`"

**Solutions**:
1. Increase `wait_sec` parameter to give nodes more startup time
2. Verify each upstream node published successfully (watch log output)
3. Check latched QoS settings in node code

### Color Detection Fails

**Symptom**: Small or large cubes not detected (wrong color reported)

**Solutions**:
1. Adjust HSV ranges for your lighting conditions (edit `_build_masks()`)
2. Increase `min_area` parameter if lots of false positives
3. Inspect camera output: `ros2 run image_tools showimage /camera/image_raw`
4. Check that cubes are within camera field of view

### Rotation Inaccurate or Overshoots

**Symptom**: Robot rotates past 90°, lands at wrong heading

**Solutions**:
1. Reduce `angular_speed` (default: 0.35 → try 0.25)
2. Increase `yaw_kp` for faster feedback response (default: 1.2 → try 1.5)
3. Decrease `yaw_tolerance` for tighter final positioning (default: 0.04 → try 0.02)

### Base Navigation Overshoots or Arrives at Wrong Position

**Symptom**: Robot moves past target location or reaches with wrong heading

**Solutions**:
1. Increase `nav_heading_kp` for stronger heading correction (default: 1.3 → try 1.6)
2. Decrease `nav_lin_tolerance` for closer final stop (default: 0.08 → try 0.05)
3. Reduce `nav_max_lin_speed` for smoother approach (default: 0.20 → try 0.15)
4. Increase `nav_turn_only_threshold` to force heading correction earlier (default: 0.18 → try 0.25)

### Grasp Fails

**Symptom**: "Failed to grasp small cube"

**Solutions**:
1. Verify small cube spawned at expected location: `(0.7, 0.0, 0.28)` table-relative
2. Check UR3 arm is properly initialized and moving
3. Inspect gripper action server connection: `ros2 action list`

## File Structure

```
pick_and_place/
├── README.md                    # This file
├── package.xml                  # ROS2 package metadata
├── setup.py                     # Setup configuration
├── setup.cfg                    # Setuptools config
├── pick_and_place/
│   ├── __init__.py
│   ├── cube_spawner.py          # Spawn cubes in Gazebo
│   ├── detect_small_cubes.py    # Detect small cube color
│   ├── pick_middle_cube.py      # Pick and navigate to target
│   ├── rotate_scan_color.py     # Scan and match large cube colors
│   ├── camera_node.py           # Camera bridge (reference)
│   └── ...
└── launch/
    └── color_match.launch.py    # Main launch file
```

## Related Packages in Workspace

This package is part of a larger omni-robot system. Related packages:

### omni_robot_description
Robot URDF and xacro files defining the omni-directional mobile base and UR3 arm kinematic structure.

### omni_description
Launch files and RViz configuration for visualizing the robot state and planning.

### omni_control
Hardware interfaces and controller configurations for base and arm actuators (motor control, joint commands).

### omni_gazebo
Gazebo simulation environment setup, physics parameters, and robot spawning configuration for gazebo_ros.

### omni_moveit2
MoveIt2 configuration packages for inverse kinematics, motion planning, and collision checking for the UR3 arm.

### omni_navigation
⚠️ **Under Development** - Navigation stack integration (SLAM, path planning, costmaps). Not yet integrated into this pick-and-place workflow.

### pick_and_place (this package)
High-level task execution: object detection, manipulation planning, and coordinated base+arm control for pick-and-place operations.

## License

Apache License 2.0 - See `package.xml` for details

## Contributing

Contributions welcome! Please ensure:
- Code follows PEP 8 style guidelines
- New features include parameter documentation
- Launch changes are backward-compatible

## Support

For issues or questions, please open an issue on GitHub or contact the maintainer.
