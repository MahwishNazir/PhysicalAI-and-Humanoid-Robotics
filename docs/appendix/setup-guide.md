---
title: "Setup Guide"
sidebar_label: "Setup Guide"
sidebar_position: 3
description: "Environment setup instructions for Physical AI & Humanoid Robotics development"
---

# Setup Guide

This guide will help you set up your development environment for the hands-on exercises throughout this book.

## Prerequisites

### Required Knowledge

- Basic Python programming (variables, functions, loops, classes)
- Command-line interface familiarity
- Basic linear algebra concepts (optional but helpful)

### System Requirements

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **RAM**: Minimum 8GB, 16GB recommended
- **Disk Space**: At least 20GB free space
- **CPU**: Modern multi-core processor (4+ cores recommended)
- **GPU**: Optional but recommended for advanced simulation (NVIDIA with CUDA support)

## Step 1: Python Environment

### Install Python 3.8+

**Ubuntu/Linux:**
```bash
sudo apt update
sudo apt install python3.10 python3.10-venv python3-pip
```

**Windows (WSL2):**
```bash
# Install WSL2 if not already installed
wsl --install -d Ubuntu-22.04

# Inside WSL2, install Python
sudo apt update
sudo apt install python3.10 python3.10-venv python3-pip
```

**macOS:**
```bash
# Using Homebrew
brew install python@3.10
```

### Create a Virtual Environment

```bash
# Create a dedicated environment for robotics
python3 -m venv ~/robotics-env

# Activate the environment
source ~/robotics-env/bin/activate  # Linux/macOS
# or
~/robotics-env/Scripts/activate  # Windows
```

### Install Essential Python Libraries

```bash
pip install --upgrade pip
pip install numpy scipy matplotlib
pip install jupyter notebook  # For interactive development
```

## Step 2: ROS 2 Installation

ROS 2 (Robot Operating System 2) is the core middleware for robot development.

### Install ROS 2 Humble (Ubuntu 22.04)

```bash
# Set up sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Set up environment (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
```

### Install ROS 2 Development Tools

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
```

### Verify ROS 2 Installation

```bash
# Check ROS 2 version
ros2 --version

# Test with a demo (run in separate terminals)
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

## Step 3: Simulation Platforms

### Option A: PyBullet (Recommended for Beginners)

PyBullet is lightweight, easy to install, and works on all platforms.

```bash
# Activate your virtual environment first
source ~/robotics-env/bin/activate

# Install PyBullet
pip install pybullet

# Test installation
python3 -c "import pybullet as p; print('PyBullet version:', p.getVersion())"
```

### Option B: Gazebo (ROS Integration)

Gazebo provides realistic physics simulation and integrates well with ROS 2.

**Install Gazebo Harmonic:**

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo
sudo apt update
sudo apt install gz-harmonic

# Install ROS 2 Gazebo bridge
sudo apt install ros-humble-ros-gz
```

**Test Gazebo:**

```bash
gz sim shapes.sdf
```

### Option C: NVIDIA Isaac Sim (Advanced, GPU Required)

Isaac Sim is a GPU-accelerated simulator for advanced robotics and AI research.

**Requirements:**
- NVIDIA GPU (RTX 2070 or better)
- Ubuntu 20.04/22.04 or Windows 10/11
- NVIDIA drivers (525.60.11 or later)

**Installation:**
1. Download from [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
2. Follow the official installation guide
3. Recommended for Module 4 (Humanoid Integration)

## Step 4: Computer Vision Libraries

For robot perception chapters (Module 2).

### Install OpenCV

```bash
# Activate virtual environment
source ~/robotics-env/bin/activate

# Install OpenCV
pip install opencv-python opencv-contrib-python

# Test installation
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
```

### Install Point Cloud Library (Optional)

For 3D vision and LiDAR processing:

```bash
sudo apt install libpcl-dev
pip install python-pcl
```

## Step 5: Robotics Libraries

### Install Robotics Toolbox

```bash
# Activate virtual environment
source ~/robotics-env/bin/activate

# Install robotics libraries
pip install roboticstoolbox-python
pip install spatialmath-python
pip install ikpy  # Inverse kinematics
```

### Verify Installation

```python
# Test script
import roboticstoolbox as rtb
import numpy as np

# Create a simple 2-DOF robot
robot = rtb.models.DH.Planar2()
print("Robot created:", robot.name)
print("Number of joints:", robot.n)
```

## Step 6: Development Tools

### Install Visual Studio Code (Recommended IDE)

```bash
# Ubuntu
sudo snap install code --classic

# Or download from https://code.visualstudio.com/
```

**Recommended VS Code Extensions:**
- Python
- ROS
- C/C++ (for ROS development)
- Jupyter
- URDF Previewer

### Install Git

```bash
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

## Step 7: Download Book Resources

```bash
# Create a workspace directory
mkdir -p ~/robotics-workspace/book-examples
cd ~/robotics-workspace/book-examples

# Download code examples (replace with actual repository)
# git clone https://github.com/your-repo/physical-ai-book-examples.git

# Set up ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Verification Checklist

Run through this checklist to ensure everything is set up correctly:

- [ ] Python 3.8+ installed and virtual environment created
- [ ] ROS 2 Humble installed and sourced
- [ ] PyBullet or Gazebo installed and tested
- [ ] OpenCV installed for vision tasks
- [ ] Robotics Toolbox installed
- [ ] IDE (VS Code) installed with extensions
- [ ] Git configured
- [ ] Code examples downloaded (when available)

## Troubleshooting

### Common Issues

**ROS 2 commands not found:**
```bash
# Make sure to source ROS 2 setup
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**PyBullet GUI not appearing:**
```bash
# Install required GUI libraries
sudo apt install python3-tk
pip install --upgrade pybullet
```

**Gazebo crashes on startup:**
```bash
# Update graphics drivers
sudo ubuntu-drivers autoinstall

# Use software rendering if needed
export LIBGL_ALWAYS_SOFTWARE=1
gz sim
```

**Permission denied errors:**
```bash
# Fix workspace permissions
sudo chown -R $USER:$USER ~/ros2_ws
sudo chown -R $USER:$USER ~/robotics-workspace
```

### Getting Help

- **ROS 2**: [ROS Discourse](https://discourse.ros.org/)
- **Gazebo**: [Gazebo Community](https://community.gazebosim.org/)
- **PyBullet**: [PyBullet Forum](https://pybullet.org/wordpress/)
- **Book Issues**: [GitHub Issues](https://github.com/your-repo/issues) (when repository is available)

## Alternative Setups

### Docker Container (Cross-Platform)

If you prefer not to install everything locally:

```bash
# Pull ROS 2 Docker image
docker pull osrf/ros:humble-desktop

# Run container with GUI support
docker run -it --rm \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  osrf/ros:humble-desktop
```

### Cloud Development (Google Colab, Replit)

For basic Python exercises without ROS 2:
- [Google Colab](https://colab.research.google.com/) - Free Jupyter notebooks with GPU
- [Replit](https://replit.com/) - Online Python environment

## Next Steps

Once your environment is set up:

1. **Test Your Setup**: Run the verification scripts from the book repository
2. **Start Module 1**: Begin with "What is ROS 2?" to understand robot architecture
3. **Join the Community**: Connect with other learners and ask questions
4. **Bookmark References**: Keep the [References](./references.md) page handy

---

**Setup complete?** Head to [Module 1: The Robotic Nervous System](../module-01-robotic-nervous-system/index.md) to begin your journey into Physical AI!
