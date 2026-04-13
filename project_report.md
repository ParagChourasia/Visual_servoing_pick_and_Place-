# Technical Report: Visual Servoing Pick-and-Place for Franka Panda

## 1. Project Overview
This project implements an industrial-grade autonomous pick-and-place pipeline for the Franka Panda (FER) robot using **ROS 2 Humble** and **Gazebo Fortress**. The system integrates vision-based object detection, a robust state machine, and MoveIt 2 for motion planning to sort objects into dynamic bins.

---

## 2. System Architecture

### 2.1 Software Stack
- **Operating System**: Linux (Ubuntu 22.04)
- **Middleware**: ROS 2 Humble
- **Simulation**: Gazebo Fortress (Ignition)
- **Motion Planning**: MoveIt 2 (OMPL, KDL Solver)
- **Perception**: OpenCV (HSV-based detection) + Depth Sensors

### 2.2 Package Breakdown
- `franka_sim_setup`: Core workspace configuration, Gazebo worlds, and launch orchestration.
- `franka_description`: Unified Robot Description Format (URDF) and XACRO files for the Franka FER.
- `franka_ros2`: ROS 2 controllers and driver interfaces.
- `libfranka`: Bottom-level C++ library for Franka hardware communication.

---

## 3. Autonomous Pipeline (Step-by-Step)

### Phase 1: Initialization & Readiness
The system performs a diagnostic check before starting. It monitors:
- **Clock**: Ensures simulation time is synchronized.
- **Vision**: Confirms the wrist camera feed is active.
- **Joints**: Verifies that the standard ROS 2 `joint_states` are being published.
- **MoveIt**: Ensures the `move_group` action server is ready for planning requests.

### Phase 2: Object Discovery (Search State)
The robot transitions to a **Home** position to survey the workbench.
1. The `object_detector` node applies HSV filters to identify colored cubes (Red, Green, Blue).
2. Once a target is centered in the wrist camera, the pixel coordinates $(u, v)$ and depth value $d$ are captured.
3. The system selects the closest target and transitions to the `SelectingObject` state.

### Phase 3: Visual Servoing 'Lite' (Approach State)
To handle simulation jitter and ensure accuracy:
1. **Initial Projection**: Converts pixel data to world coordinates $(X, Y, Z)$ using camera intrinsics and TF transforms.
2. **Coarse Approach**: Moves the end-effector to a safe altitude $(Z=0.1m)$ above the target.
3. **Refinement Pause**: The robot stops for 1 second to allow the vision system to re-calibrate.
4. **Fine Alignment**: The target position is updated based on the new, close-range view, correcting for any projection errors.

### Phase 4: Precision Grasping
1. **Dive**: The arm moves down to the exact grasp height $(Z=0.04m)$.
2. **Action**: The gripper closes with maximum tension.
3. **Hardened Grasp (Safety)**: To prevent physics-engine "slips," the `DetachableJoint` plugin is triggered to logically lock the object to the gripper fingers.
4. **Retraction**: The arm lifts the object to a safe travel height.

### Phase 5: Verification & Delivery
1. **Grasp Verification**: The system checks joint aperture and tactile contact sensors. If the gripper is fully closed (empty), it resets the mission.
2. **Dynamic Bin Lookup**: The robot queries the locations of the sorting bins (Red/Green/Blue).
3. **Delivery**: MoveIt 2 plans a collision-free path to the target bin.
4. **Release**: The `DetachableJoint` is released, the gripper opens, and the robot returns to **Home** to sort the next object.

---

## 4. Key Innovations
- **Jitter-Resistant Control**: Hybrid use of MoveIt 2 for global navigation and direct coordinate updates for local precision.
- **Dynamic Bin Tracking**: Bin locations are not hardcoded; they are discovered at runtime, allowing for flexible sorting environments.
- **Diagnostic Dashboard**: Real-time feedback in the terminal and RViz overlays for immediate debugging of detection issues.

## 5. How to Run
```bash
# 1. Source the workspace
source install/setup.bash

# 2. Launch the full simulation
ros2 launch franka_sim_setup pick_and_place.launch.py

# 3. Monitor the State Machine
# The terminal will display the current state and mission progress.
```

---
**Author**: Parag Chourasia
**Date**: April 2026
