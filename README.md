# VS_manipulator: Franka Panda Visual Servoing & Pick-and-Place

A professional ROS 2 Humble implementation for autonomous pick-and-place operations using the Franka Panda robot in Gazebo Fortress. This project features integrated visual servoing, hardened control loops, and MoveIt 2 integration for robust manipulation tasks.

## Key Features
- **ROS 2 Humble Integration**: Modern ROS 2 architecture for distributed robotic control.
- **Gazebo Fortress (Ignition) Simulation**: High-fidelity simulation using the latest Gazebo tools.
- **Autonomous Pick-and-Place**: Full state machine for identifying, grasping, and relocating objects.
- **Advanced Grasping**: Uses the `DetachableJoint` plugin for reliable physics-based grasping.
- **MoveIt 2 Support**: Optimized motion planning and IK solver configurations for the Franka arm.
- **Visual Servoing Support**: Integration for camera-based feedback and fine-grained control.

## Workspace Structure
- `src/franka_description`: Robot descriptions and URDF/XACRO files for Franka FER/Panda.
- `src/franka_sim_setup`: Configuration, worlds, and core launch files for the simulation.
- `src/franka_ros2`: ROS 2 controllers and interface nodes for the Franka robot.
- `src/libfranka`: Core C++ library for communication with the Franka hardware.

## Prerequisites
- **ROS 2 Humble**
- **Gazebo Fortress**
- **MoveIt 2**
- **ros_gz** (ROS-Gazebo bridge)

## Build Instructions
Clone this repository into your `colcon` workspace `src` folder (if not already there) and run:
```bash
cd VS_manipulator
colcon build --symlink-install
source install/setup.bash
```

## Launching the Simulation
To start the pick-and-place simulation:
```bash
ros2 launch franka_sim_setup pick_and_place.launch.py
```

## Authors
- **Parag Chourasia** - [GitHub](https://github.com/ParagChourasia)
