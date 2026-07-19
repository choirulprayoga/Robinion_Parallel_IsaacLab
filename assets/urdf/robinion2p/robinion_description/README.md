# Robinion Description

The `robinion_description` is a ROS 2 robot description package for the humanoid robot of AIRO Lab at TKU. It contains the robot's Xacro/URDF model, STL mesh files, RViz configuration, and a launch file.

## Package Overview

This package is intended for robot model visualization and URDF/Xacro development. The model is organized into separate Xacro files for the body, head, arms, legs, and ROS 2 control interface.

## Requirements

Tested or intended for ROS 2. The package should work with recent ROS 2 Jazzy, provided the required packages are installed.

Required ROS 2 packages:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2
```

If you use ROS 2 control simulation or hardware integration later, install the relevant `ros2_control` packages for your ROS 2 distribution.

## Installation

Create a ROS 2 workspace if you do not already have one:

```bash
mkdir -p ~/robinion_ws/src
cd ~/robinion_ws/src
```

clone this package into the `src` folder:

```bash
git clone https://github.com/TKU-AIRO-Lab/robinion_description.git
```

Build the workspace:

```bash
cd ~/robinion_ws
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

## Usage

### Visualize the Robot in RViz2

Run:

```bash
ros2 launch robinion_description desciption.launch.py
```