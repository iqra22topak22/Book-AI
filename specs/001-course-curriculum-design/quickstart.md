# Quickstart Guide: Physical AI & Humanoid Robotics Course

## Overview
This guide provides a rapid setup pathway for educators and students to begin with the Physical AI & Humanoid Robotics course. The course combines ROS 2, NVIDIA Isaac, simulation environments, and real hardware to teach embodied intelligence concepts.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (primary), Windows 10/11 with WSL2, or macOS
- **RAM**: Minimum 16GB (32GB+ recommended)
- **GPU**: NVIDIA GPU with 8GB+ VRAM (RTX 3060 or better), RTX 4090 for full simulation performance
- **Disk Space**: 50GB+ available (100GB+ for complete setup)

### Software Requirements
- Git for version control
- Docker (for isolated environments)
- Python 3.10+ with pip
- ROS 2 Humble Hawksbill

## Environment Setup

### 1. System Preparation
```bash
# Install system dependencies
sudo apt update && sudo apt install -y python3-pip python3-colcon-common-extensions

# Install Node.js (for Docusaurus documentation)
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### 2. ROS 2 Installation
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-nav2-bringup

# Setup ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Course Repository Setup
```bash
# Clone the course repository
git clone https://github.com/your-organization/physical-ai-course.git
cd physical-ai-course

# Install Python dependencies
pip3 install -r requirements.txt

# Setup ROS workspaces
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Simulation Environment Setup

#### Gazebo Installation
```bash
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

#### NVIDIA Isaac Sim Setup
1. Download Isaac Sim from NVIDIA Developer Portal
2. Follow installation instructions for your operating system
3. Set up the Isaac ROS Development Environment:
```bash
# Clone Isaac ROS repositories
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git -b humble
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bringup.git -b humble
```

### 5. Docusaurus Documentation Setup
```bash
cd docusaurus
npm install
npm run start
```

## Module 1: ROS 2 Nervous System
- Navigate to `docusaurus/docs/module-1-ros-nervous-system`
- Complete the tutorials on nodes, topics, services, and URDF
- Practice with the rclpy bridge examples
- Run the provided simulation launch files

## Module 2: Digital Twin
- Navigate to `docusaurus/docs/module-2-digital-twin`
- Set up Gazebo and Unity environments
- Explore physics simulation and sensor integration
- Complete the sensor fusion exercises

## Module 3: NVIDIA Isaac Brain
- Navigate to `docusaurus/docs/module-3-nvidia-isaac-brain`
- Install Isaac Sim and Isaac ROS packages
- Work with VSLAM and Nav2 navigation stack
- Implement localization and mapping exercises

## Module 4: Vision-Language-Action
- Navigate to `docusaurus/docs/module-4-vision-language-action`
- Integrate Whisper for voice processing
- Implement LLM-based planning systems
- Complete the capstone project: Voice → Plan → Navigate → Perceive → Manipulate

## Hardware Integration
### Jetson Orin Setup
1. Flash Jetson Orin with appropriate image
2. Configure ROS 2 environment
3. Connect sensors (RealSense, IMU, mic array)
4. Calibrate sensors following the provided documentation

### Robot Platform Setup
- For Unitree robots: Follow manufacturer ROS integration guide
- For Hiwonder robots: Install provided ROS packages
- Calibrate robot kinematics and dynamics

## Cloud Ether Lab Integration
1. Configure AWS CLI with your credentials
2. Set up EC2 instances with g5/g6e configuration
3. Deploy Omniverse environment
4. Connect to remote simulation from local development environment

## Running the Capstone Project
```bash
# Start the complete system
ros2 launch physical_ai_capstone capstone_project.launch.py

# Command the robot via voice
ros2 run voice_command voice_input_node
```

## Validation and Testing
- Run all module tests to validate your setup:
```bash
cd tests/
python3 -m pytest
```

## Getting Help
- Documentation: Check the Docusaurus site at localhost:3000
- Discussion Forum: Post questions in the course's designated forum
- Troubleshooting: Refer to the troubleshooting guide in the documentation

## Next Steps
1. Complete Module 1: ROS 2 Nervous System
2. Progress through each module sequentially
3. Participate in the capstone project
4. Consider contributing to the course content based on your findings