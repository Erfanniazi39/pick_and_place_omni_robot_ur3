# Omni-Robot System with Pick-and-Place Task

A complete ROS2-based autonomous robotic system featuring an omni-directional mobile base equipped with a UR3 collaborative arm. The system is designed to autonomously pick small colored cubes and place them next to matching-colored larger cubes.


## Project Overview

This workspace contains a fully integrated robotic manipulation system with:
- **Hardware**: Omni-directional mobile base (omni wheels) + UR3 robotic arm
- **Simulation**: Gazebo Sim with physics-based object interaction
- **Perception**: OpenCV-based color detection and vision processing
- **Planning**: MoveIt2 for arm inverse kinematics and motion planning
- **Control**: ROS2 control framework for base and arm actuation
- **Task**: Autonomous color-matching pick-and-place workflow

## System Architecture

```
Physical/Simulation Layer
├── omni_gazebo
│   └── Gazebo environment, robot spawning, physics simulation
│
Hardware Interface Layer
├── omni_control
│   └── Motor controllers, joint commands, actuator drivers
│
Robot Description Layer
├── omni_robot_description
├── omni_description
│   └── URDF/Xacro models, kinematics, RViz visualization
│
Planning & Navigation Layer
├── omni_moveit2
│   └── Arm inverse kinematics, motion planning, collision checking
└── omni_navigation (Under Development)
    └── SLAM, path planning, navigation stack
│
Task Execution Layer
└── pick_and_place
    ├── Cube spawning & detection
    ├── Arm grasping & manipulation
    ├── Base navigation & positioning
    └── Color matching workflow
```

## Package Descriptions

### omni_robot_description
Defines the robot's structure (shape, size, joints). Contains the 3D model files and tells ROS what the robot looks like.

### omni_description
Shows the robot on the screen in RViz (visualization tool). Displays the robot's current position and arm configuration.

### omni_control
Controls the robot's motors and actuators. Handles commands like "move forward" and "rotate arm" and sends them to the motors.

### omni_gazebo
Creates the virtual environment where the robot operates. Runs the physics simulation, spawns cubes, and simulates camera/sensors.

### omni_moveit2
Plans how the arm should move. Calculates arm positions, checks for collisions, and generates safe motion paths.

### omni_navigation ⚠️ (Under Development)
Will handle robot navigation (mapping, path planning). Not yet completed—reserved for future features.

### pick_and_place
The main task program. Detects colored cubes, picks them up, and places them next to matching colors.

**Workflow**:
1. Spawn test objects (1 small cube, 3 large cubes)
2. Detect small cube color via camera
3. Pick small cube using arm
4. Rotate base to scan large cubes
5. Match colors and locate target
6. Navigate base to target position
7. Place small cube beside matching large cube

