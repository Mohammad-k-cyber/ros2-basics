

---

# ROS2 Learning Project (Turtlesim + URDF Robot Arm)

A comprehensive ROS2 workspace demonstrating fundamental concepts in two parts:

1. **Turtlesim Learning Package (`turtlesim_test`)**
   Covers publishers, services, parameters, actions, keyboard control, and launch files.

2. **Robot Description Package (`my_robot_description`)**
   Defines a 3-DOF robotic arm using your `my_robot_arm.urdf` file, viewable in RViz2, and controlled via `joint_state_publisher_gui`.

---

## Table of Contents

* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Project Overview](#project-overview)
* [Project Structure](#project-structure)
* [Setup Instructions](#setup-instructions)
* [Running the Project](#running-the-project)
* [File Descriptions](#file-descriptions)
* [Learning Outcomes](#learning-outcomes)
* [Troubleshooting](#troubleshooting)
* [Next Steps](#next-steps)
* [Additional Resources](#additional-resources)

---

## Prerequisites

* ROS2 Humble (or compatible version)
* Python 3.10+
* Ubuntu 22.04 (recommended)
* `colcon` build tool
* RViz2 installed (`sudo apt install ros-humble-rviz2`)

---

## Installation

### 1. Clone Repository

```bash
cd ~
git clone https://github.com/Mohammad-k-cyber/ros2-basics.git
cd ros2-basics
```

### 2. Install Dependencies

```bash
sudo apt update
sudo apt install ros-humble-turtlesim ros-humble-rviz2 ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-xacro
```

### 3. Build Workspace

```bash
colcon build
source install/setup.bash
```

---

## Project Overview

### Part 1: **Turtlesim Test Package**

| Component               | Concept             | Description                       |
| ----------------------- | ------------------- | --------------------------------- |
| `turtle_publisher.py`   | Publisher           | Moves turtle in circular patterns |
| `turtle_service.py`     | Service             | Clears the background trail       |
| `turtle_param.py`       | Parameters          | Changes background colors         |
| `turtle_action.py`      | Action              | Rotates turtle to specific angles |
| `turtle_keyboard.py`    | Interactive Control | WASD turtle movement              |
| `turtle_test.launch.py` | Launch              | Coordinates multiple nodes        |

### Part 2: **My Robot Description**

| File                             | Purpose                                                               |
| -------------------------------- | --------------------------------------------------------------------- |
| `urdf/my_robot_arm.urdf`         | Defines 3-DOF robot arm (black + blue links)                          |
| `launch/display.launch.py`       | Starts `robot_state_publisher`, `joint_state_publisher_gui`, and RViz |
| `rviz/display.rviz`              | Preconfigured RViz2 visualization                                     |
| `package.xml` / `CMakeLists.txt` | Dependencies and build setup                                          |

---

## Project Structure

```
ros2-basics/
├── src/
│   ├── turtlesim_test/                # Part 1: Turtlesim learning
│   │   ├── launch/
│   │   │   └── turtle_test.launch.py
│   │   ├── turtlesim_test/
│   │   │   ├── turtle_publisher.py
│   │   │   ├── turtle_service.py
│   │   │   ├── turtle_param.py
│   │   │   ├── turtle_action.py
│   │   │   └── turtle_keyboard.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── my_robot_description/          # Part 2: Robot description
│       ├── urdf/my_robot_arm.urdf
│       ├── launch/display.launch.py
│       ├── rviz/display.rviz
│       ├── package.xml
│       └── CMakeLists.txt
├── build/                             # Auto-generated
├── install/                           # Auto-generated
└── log/                               # Auto-generated
```

---

## Running the Project

### 1. Turtlesim Nodes

**Example: Publisher**

```bash
# Terminal 1
ros2 run turtlesim turtlesim_node
# Terminal 2
ros2 run turtlesim_test turtle_publisher
```

**Example: Launch file**

```bash
ros2 launch turtlesim_test turtle_test.launch.py
```

---

### 2. Robot Arm in RViz2

```bash
ros2 launch my_robot_description display.launch.py
```

* In RViz, set **Fixed Frame** = `base_link`
* Use sliders in **Joint State Publisher GUI** to move the 3 DOF arm
* Black + blue robot links should appear in 3D

---

## File Descriptions

### Turtlesim Test

* **Publisher**: Sends velocity commands
* **Service**: Calls `/clear` to wipe screen
* **Parameter**: Changes background colors dynamically
* **Action**: Rotates turtle using long-running goal
* **Keyboard**: Real-time WASD teleop

### My Robot Description

* **URDF**: Robot model (`my_robot_arm.urdf`) with 3 revolute joints
* **Launch**: Combines `robot_state_publisher` + RViz2
* **RViz Config**: Displays model and TF tree

---

## Learning Outcomes

* ROS2 communication (Topics, Services, Actions, Parameters)
* Launch file usage
* Creating URDF robot models
* Visualizing robots in RViz2
* Joint control with GUI

---

## Troubleshooting

**Robot not visible in RViz**

* Check Fixed Frame = `base_link`
* Ensure `joint_state_publisher_gui` is running

**Package not found**

```bash
source install/setup.bash
```

**URDF errors**

```bash
check_urdf src/my_robot_description/urdf/my_robot_arm.urdf
```

---

## Next Steps

* Add custom meshes for the robot
* Convert URDF to Xacro for reusability
* Implement RViz2 TF frames monitoring
* Extend arm to 4/5 DOF

---

## Additional Resources

* [ROS2 Documentation](https://docs.ros.org/en/humble/)
* [URDF Tutorials](https://wiki.ros.org/urdf/Tutorials)
* [Turtlesim Docs](https://github.com/ros/ros_tutorials/tree/humble/turtlesim)

---

**Maintainer**: CyberAi
**License**: MIT
**Version**: 0.1.0
**GitHub Repository**: [https://github.com/Mohammad-k-cyber/ros2-basics](https://github.com/Mohammad-k-cyber/ros2-basics)

---


