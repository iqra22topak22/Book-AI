# Lesson 1.1: Introduction to ROS 2 Architecture

## Overview
This lesson introduces the fundamental concepts of ROS 2 architecture, including the differences from ROS 1 and the key components that make up the ROS 2 ecosystem.

## Learning Objectives
By the end of this lesson, students will be able to:
- Explain the differences between ROS 1 and ROS 2
- Identify the core components of ROS 2
- Understand the DDS-based communication layer
- Set up a basic ROS 2 workspace

## Topics Covered
1. ROS 1 vs ROS 2 Overview
2. ROS 2 Core Components
3. DDS and Middleware Layer
4. Setting up ROS 2 Environment

## 1. ROS 1 vs ROS 2 Overview

### Key Differences
- **Middleware**: ROS 1 uses a custom transport layer, while ROS 2 uses DDS (Data Distribution Service)
- **Real-time Support**: ROS 2 provides better real-time capabilities
- **Distributed Architecture**: ROS 2 does not require a master node
- **Multi-platform**: ROS 2 supports Linux, Windows, and macOS natively
- **Security**: ROS 2 includes built-in security capabilities
- **Quality of Service (QoS)**: ROS 2 has explicit QoS policies for message delivery

### Why ROS 2?
ROS 2 addresses key limitations of ROS 1:
- Master single point of failure
- Lack of real-time support
- Limited multi-robot support
- Security concerns
- Platform limitations

## 2. ROS 2 Core Components

### Nodes
- Processes that perform computation
- Communicate with other nodes through:
  - Topics (publish/subscribe)
  - Services (request/reply)
  - Actions (goal/cancel/result feedback)

### Topics
- Asynchronous, many-to-many communication
- Publishers send messages to topics
- Subscribers receive messages from topics
- Implemented using DDS

### Services
- Synchronous, request/reply communication
- Client sends request, server sends response
- Implemented using DDS request/reply patterns

### Actions
- Asynchronous goal-oriented communication
- For long-running tasks with feedback
- Includes goal, feedback, and result messages

## 3. DDS and Middleware Layer

### Data Distribution Service (DDS)
- Standard for real-time, distributed systems
- Provides:
  - Discovery of nodes
  - Data transportation
  - Quality of Service (QoS) policies
  - Language interoperability

### Middleware
- Abstraction layer between ROS 2 and DDS
- Allows using different DDS implementations
- Examples: Fast DDS, Cyclone DDS, RTI Connext DDS

## 4. Setting up ROS 2 Environment

### Prerequisites
- Ubuntu 22.04 LTS (or supported platform)
- Sufficient disk space (2-5 GB)

### Installation
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
```

### Environment Setup
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to your bashrc to auto-source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Basic Commands
```bash
# Check ROS version
echo $ROS_DISTRO

# List available packages
ros2 pkg list

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Exercise
1. Install ROS 2 Humble Hawksbill on your system
2. Verify the installation by running `echo $ROS_DISTRO`
3. Create a basic workspace and build it using colcon

## Summary
This lesson covered the fundamental differences between ROS 1 and ROS 2, the core components of the ROS 2 architecture, and how to set up your development environment. The next lesson will explore nodes and packages in detail.