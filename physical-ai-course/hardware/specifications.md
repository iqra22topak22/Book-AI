# Hardware Architecture Specification

## Overview
This document outlines the hardware architecture specification for the Physical AI & Humanoid Robotics course, detailing the required components for a practical learning environment.

## Workstation Specifications

### RTX Workstations
- **GPU**: NVIDIA RTX 4090 or equivalent (24GB+ VRAM recommended)
- **CPU**: 16+ core processor (AMD Ryzen 9 or Intel i9)
- **RAM**: 64GB+ DDR4/DDR5
- **Storage**: 2TB+ NVMe SSD for simulation environments
- **OS**: Ubuntu 22.04 LTS (primary), with Windows 10/11 WSL2 support

### Jetson Edge Kits
- **Platform**: NVIDIA Jetson AGX Orin Developer Kit
- **CPU**: 12-core ARM Cortex-A78AE v8.2 64-bit CPU
- **GPU**: 2048-core NVIDIA Ampere architecture GPU with 64 Tensor Cores
- **Memory**: 32GB 256-bit LPDDR5/6000 RAM
- **Connectivity**: Dual-band WiFi 6, Bluetooth 5.2, 2.5GBASE-T Ethernet

## Robot Platforms

### Unitree Robots
- **Unitree Go2**: Quadruped robot with 2D LiDAR, RGB camera
- **Unitree G1**: Humanoid robot with multiple sensors and actuators
- **ROS Compatibility**: Full ROS 2 Humble Hawksbill integration
- **Sensors**: IMU, force/torque sensors, cameras
- **Actuators**: High-torque servo motors with precise control

### Hiwonder Robots
- Alternative robot platform for educational settings
- ROS 2 integration
- Appropriate for laboratory environments

## Sensors

### Intel RealSense
- **Model**: D455 or D435i
- **Features**: RGB-D camera, built-in IMU
- **Applications**: Perception, mapping, object recognition

### IMU Sensors
- **Specifications**: 3-axis accelerometer, gyroscope, magnetometer
- **Accuracy**: Sufficient for robot state estimation
- **Integration**: ROS 2 sensor_msgs/Imu topic compatibility

### Microphone Array
- **Configuration**: Multi-channel audio capture
- **Application**: Voice command processing
- **Integration**: Compatible with Whisper API for speech recognition

## Edge Computing Devices

### Jetson Orin Specifications
- **GPU**: 2048-core NVIDIA Ampere architecture GPU
- **CUDA Cores**: Equivalent to high-end discrete GPU
- **DL Accelerator Cores**: 32 Tensor Cores
- **Memory**: 32GB 256-bit LPDDR5/6000 RAM
- **Performance**: 275 TOPS AI performance

## Cloud Infrastructure

### On-Premise GPU Lab
- RTX 4090 workstations for intensive simulation
- Shared storage for simulation assets
- Network infrastructure for robot communication

### Ether Lab (Cloud)
- AWS g5/g6e instances for compute-intensive tasks
- NVIDIA GPU instances for simulation and inference
- Omniverse for advanced physics simulation
- Connection protocols for remote lab access

## Compatibility Requirements

- All components must support ROS 2 Humble Hawksbill
- Communication protocols must be standardized
- Hardware abstraction layer for cross-platform compatibility
- Real-time performance requirements for control systems

## Validation Criteria

- All hardware components successfully integrated with ROS 2
- Performance benchmarks met for simulation and real-time control
- Safety protocols implemented for physical robot operation
- Network communication reliable between all components