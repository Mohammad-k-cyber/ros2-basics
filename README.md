# ROS2 Turtlesim Learning Project

A comprehensive ROS2 project demonstrating fundamental concepts including publishers, services, parameters, actions, and keyboard control using the turtlesim package.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Project Overview](#project-overview)
- [Project Structure](#project-structure)
- [Setup Instructions](#setup-instructions)
- [Running the Project](#running-the-project)
- [File Descriptions](#file-descriptions)
- [Learning Outcomes](#learning-outcomes)
- [Troubleshooting](#troubleshooting)

## Prerequisites

- ROS2 Humble (or compatible version)
- Python 3.10+
- Ubuntu 22.04 (recommended)
- colcon build tool

## Installation

### 1. Install Turtlesim Package

Update your package list and install turtlesim:

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

### 2. Verify Installation

To check if the package is installed correctly, run the following command, which should return a list of turtlesim's executables:

```bash
ros2 pkg executables turtlesim
```

Expected output:
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

### 3. Test Turtlesim

Start turtlesim to verify it works:

```bash
ros2 run turtlesim turtlesim_node
```

Expected output:
```
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```

You should see a blue window with a turtle in the center. Press `Ctrl+C` to close it.

## Project Overview

This project implements various ROS2 communication patterns and concepts:

| Component | ROS2 Concept | Description |
|-----------|-------------|-------------|
| **turtle_publisher.py** | Publisher/Subscriber | Moves turtle in circular patterns |
| **turtle_service.py** | Service Client | Clears the background trail |
| **turtle_param.py** | Parameter Management | Changes background colors |
| **turtle_action.py** | Action Client | Rotates turtle to specific angles |
| **turtle_keyboard.py** | Interactive Control | Manual WASD turtle control |
| **turtle_test.launch.py** | Launch System | Coordinates multiple nodes |

## Project Structure

```
ros2_basics/
├── src/
│   └── turtlesim_test/
│       ├── launch/
│       │   └── turtle_test.launch.py    # Multi-node launcher
│       ├── turtlesim_test/
│       │   ├── __init__.py             # Python package marker
│       │   ├── turtle_publisher.py     # Publisher demo (circular movement)
│       │   ├── turtle_service.py       # Service demo (clear background)
│       │   ├── turtle_param.py         # Parameter demo (color change)
│       │   ├── turtle_action.py        # Action demo (rotation)
│       │   └── turtle_keyboard.py      # Keyboard control (WASD)
│       ├── package.xml                 # Package dependencies
│       └── setup.py                    # Installation configuration
├── build/                              # Build artifacts (auto-generated)
├── install/                            # Installation files (auto-generated)
└── log/                                # Build logs (auto-generated)
```

## Setup Instructions

### 1. Create Workspace

```bash
mkdir -p ros2_basics/src
cd ros2_basics/
```

### 2. Create Package

```bash
cd src/
ros2 pkg create --build-type ament_python turtlesim_test \
  --dependencies rclpy geometry_msgs turtlesim std_srvs
```

### 3. Add Project Files

Copy all the provided Python files to their respective directories:
- Copy Python node files to `src/turtlesim_test/turtlesim_test/`
- Copy launch file to `src/turtlesim_test/launch/`
- Replace `package.xml` and `setup.py` with provided versions

### 4. Build and Source

```bash
cd ~/ros2_basics
colcon build
source install/setup.bash
```

## Running the Project

**Always source the workspace first:**
```bash
cd ~/ros2_basics
source install/setup.bash
```

### Individual Node Examples

#### 1. Publisher Demo (Circular Movement)
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Start publisher
ros2 run turtlesim_test turtle_publisher
```

#### 2. Service Demo (Clear Background)
```bash
# Terminal 1: Start turtlesim  
ros2 run turtlesim turtlesim_node

# Terminal 2: Clear background
ros2 run turtlesim_test turtle_service
```

#### 3. Parameter Demo (Change Colors)
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Change background to light blue
ros2 run turtlesim_test turtle_param
```

#### 4. Action Demo (Rotate 90°)
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Rotate turtle
ros2 run turtlesim_test turtle_action
```

#### 5. Keyboard Control (WASD)
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Keyboard control
ros2 run turtlesim_test turtle_keyboard
# Use W/A/S/D to move, Q to quit
```

### Launch File Demo (Automated)

Start both turtlesim and publisher automatically:
```bash
ros2 launch turtlesim_test turtle_test.launch.py
```

## File Descriptions

### Core Nodes

- **`turtle_publisher.py`**: Demonstrates continuous publishing of velocity commands, making the turtle move in circles
- **`turtle_service.py`**: Shows one-time service calls to clear the turtle's background trail
- **`turtle_param.py`**: Illustrates parameter management by changing background colors
- **`turtle_action.py`**: Demonstrates long-running actions with the turtle rotation (exits after completion)
- **`turtle_keyboard.py`**: Provides real-time interactive control using keyboard input

### Configuration Files

- **`turtle_test.launch.py`**: Coordinates multiple nodes, starting both turtlesim and publisher together
- **`setup.py`**: Defines package installation, executables, and data file locations
- **`package.xml`**: Specifies dependencies and package metadata for ROS2

## Learning Outcomes

After completing this project, you will understand:

### ROS2 Communication Patterns
- **Topics**: Continuous data streaming (velocity commands)
- **Services**: Request-response communication (clear background)
- **Actions**: Long-running tasks with feedback (rotation)
- **Parameters**: Runtime configuration (colors, speeds)

### Development Skills
- Package creation and structure
- Python-based ROS2 node development
- Build system usage with colcon
- Launch file coordination
- Real-time interactive programming

### System Integration
- Multi-node coordination
- Dependency management
- Installation and distribution

## Advanced Usage

### Custom Parameters
```bash
# Run keyboard control with 2x speed
ros2 run turtlesim_test turtle_keyboard --ros-args -p speed:=2.0

# Change background color manually
ros2 param set /turtlesim background_r 255
ros2 param set /turtlesim background_g 0
ros2 param set /turtlesim background_b 0
```

### Monitoring and Debugging
```bash
# Monitor velocity commands
ros2 topic echo /turtle1/cmd_vel

# List all active nodes
ros2 node list

# Check available services
ros2 service list

# Monitor action status
ros2 action list
```

## Troubleshooting

### Common Issues

**Issue**: `Package 'turtlesim_test' not found`
```bash
# Solution: Source the workspace
cd ~/ros2_basics
source install/setup.bash
```

**Issue**: `Launch file not found`
```bash
# Solution: Rebuild the package
colcon build --packages-select turtlesim_test
source install/setup.bash
```

**Issue**: `Permission denied` on Python files
```bash
# Solution: Make files executable
chmod +x src/turtlesim_test/turtlesim_test/*.py
```

**Issue**: Node doesn't exit after action completion
- This is expected behavior for the action client - it waits for more actions
- Use `Ctrl+C` to exit or use the improved version that exits automatically

### Build Warnings
- `UserWarning: Unknown distribution option: 'tests_require'` - This is harmless
- `pkg_resources is deprecated` - This is a system warning and can be ignored

## Next Steps

1. **Modify Movement Patterns**: Change velocities in `turtle_publisher.py`
2. **Add New Services**: Explore other turtlesim services
3. **Create Custom Actions**: Implement complex movement sequences
4. **Advanced Launch Files**: Add conditions and parameters
5. **Add Subscribers**: Create nodes that read turtle pose
6. **Multi-turtle Control**: Spawn and control multiple turtles

## Additional Resources

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Turtlesim Package Documentation](https://github.com/ros/ros_tutorials/tree/humble/turtlesim)

---

**License**: MIT  
**Maintainer**: cyberai  
**Package Version**: 0.0.0
