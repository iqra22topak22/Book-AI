# Jetson Orin Setup Guide

## Overview
This guide provides instructions for setting up the NVIDIA Jetson AGX Orin Developer Kit for the Physical AI & Humanoid Robotics course. The Jetson Orin serves as an edge computing platform for running AI workloads on robots.

## Hardware Requirements

### Jetson AGX Orin Developer Kit Specifications
- **GPU**: 2048-core NVIDIA Ampere architecture GPU with 64 Tensor Cores
- **CPU**: 12-core ARM Cortex-A78AE v8.2 64-bit CPU
- **Memory**: 32GB 256-bit LPDDR5/6000 RAM
- **Storage**: 64GB eMMC
- **Connectors**: Multiple USB, Ethernet, HDMI, etc.
- **Power**: 19V DC input (adapter included)

### Additional Requirements
- 19V/65W (or higher) power adapter
- Micro-USB or USB-C cable for flashing
- Computer with Ubuntu 18.04 or 20.04 for flashing
- Internet connection for updates

## Initial Setup

### 1. Unboxing and Physical Setup
1. Remove Jetson Orin from anti-static bag
2. Install provided thermal solution (heatsink and fan) if not pre-installed
3. Connect power adapter to the barrel jack (not USB-C for power)
4. Connect micro-USB to your host computer for initial setup

### 2. Host Computer Preparation
1. On your host computer (Ubuntu 18.04 or 20.04), install required tools:
```bash
sudo apt update
sudo apt install -y python3-pip python3-dev
pip3 install jetson-stats
```

2. Install NVIDIA SDK Manager:
```bash
# Download from NVIDIA Developer website
# Follow installation instructions provided by NVIDIA
```

### 3. Flashing Jetson Orin
1. Launch NVIDIA SDK Manager
2. Sign in with your NVIDIA Developer account
3. Select Jetson AGX Orin target
4. Connect Jetson to host via micro-USB
5. Put Jetson in recovery mode (hold REC button while plugging in power)
6. SDK Manager will detect and flash the device
7. Wait for flashing to complete (this takes 30+ minutes)

## Software Configuration

### 1. Initial Configuration
After flashing completes and Jetson boots:

1. Complete the initial Ubuntu setup wizard
2. Set up user account with sudo privileges
3. Connect to Wi-Fi or Ethernet for updates
4. Update system packages:
```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Jetson-Specific Tools
1. Install jetson-stats for monitoring:
```bash
sudo -H pip3 install -U jetson-stats
sudo jtop
```

2. Verify Jetson board information:
```bash
sudo python3 -c "from jtop import jtop; print(jtop().board)"
```

### 3. JetPack SDK Verification
1. Check JetPack version:
```bash
cat /etc/nv_tegra_release
```

2. Verify CUDA installation:
```bash
nvcc --version
nvidia-smi
```

### 4. Container Support
1. Install Docker:
```bash
curl -sSL https://get.docker.com/ | sh
sudo usermod -aG docker $USER
```

2. Install NVIDIA Container Toolkit:
```bash
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker
```

3. Test Docker with GPU:
```bash
docker run --rm --gpus all nvcr.io/nvidia/cuda:12.0-devel-ubuntu20.04 nvidia-smi
```

## ROS 2 Installation

### 1. Base ROS 2 Installation
1. Set locale:
```bash
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
```

2. Add ROS 2 repository:
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

3. Install ROS 2 Humble Hawksbill:
```bash
sudo apt update
sudo apt install -y ros-humble-ros-base
```

4. Setup ROS 2 environment:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Additional ROS Packages for Robotics
1. Install essential robotics packages:
```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-tf2-tools \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-vision-opencv \
  python3-rosdep \
  python3-colcon-common-extensions
```

2. Install camera and sensor packages:
```bash
sudo apt install -y \
  ros-humble-usb-cam \
  ros-humble-hardware-interface \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

## Python Environment Setup

### 1. Install Python Packages
```bash
pip3 install --upgrade pip
pip3 install numpy scipy matplotlib
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install opencv-python openai-whisper
```

### 2. Install Jetson Inference
1. Clone the repository:
```bash
cd ~/
git clone https://github.com/dusty-nv/jetson-inference
cd jetson-inference
git submodule update --init
```

2. Build the project:
```bash
mkdir build
cd build
cmake ../
make -j$(nproc)
```

## Network Configuration

### 1. Ethernet Setup
1. Set up static IP for robot communication:
```bash
# Edit network configuration
sudo nano /etc/netplan/01-network-manager-all.yaml
```

Example configuration:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.50/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

2. Apply network configuration:
```bash
sudo netplan apply
```

### 2. ROS 2 Network Configuration
1. Set up ROS 2 domain ID to avoid interference:
```bash
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
```

2. For multi-robot systems, set up DDS configuration:
```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

## Performance Optimization

### 1. Jetson Clocks
1. To run at maximum performance:
```bash
sudo jetson_clocks
```

2. To check current power mode:
```bash
sudo nvpmodel -q
```

3. To set high performance mode:
```bash
sudo nvpmodel -m 0
```

### 2. Thermal Management
1. Install thermal tools:
```bash
sudo apt install -y thermald
```

2. Monitor temperatures:
```bash
sudo tegrastats  # Run in another terminal
# Or use jtop
jtop
```

### 3. Memory Management
1. Add swap space for memory-intensive tasks:
```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## Camera Setup

### 1. Connect and Test Camera
1. Connect CSI camera or USB camera
2. For CSI camera, check if detected:
```bash
# Check camera detection
ls /dev/video*
```

3. Test camera with v4l2:
```bash
v4l2-ctl --list-devices
```

### 2. Camera Integration with ROS 2
1. Install camera drivers:
```bash
sudo apt install -y v4l-utils
```

2. Use usb_cam package for USB cameras:
```bash
# Add to your ROS 2 workspace and build
git clone https://github.com/ros-drivers/usb_cam.git
```

## Testing and Validation

### 1. Hardware Validation
1. Check GPU status:
```bash
nvidia-smi
```

2. Check CPU and thermal status:
```bash
sudo tegrastats
```

3. Verify memory and storage:
```bash
free -h
df -h
```

### 2. ROS 2 Validation
1. Test basic ROS 2 functionality:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

2. Run a simple test:
```bash
ros2 run demo_nodes_cpp talker
# In another terminal
ros2 run demo_nodes_cpp listener
```

### 3. AI Workload Testing
1. Test PyTorch on GPU:
```python
import torch
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"GPU count: {torch.cuda.device_count()}")
if torch.cuda.is_available():
    print(f"Current GPU: {torch.cuda.get_device_name(0)}")
```

## Troubleshooting

### Common Issues
1. **Jetson won't boot**: Check power supply and thermal solution installation
2. **CUDA not available**: Ensure proper JetPack installation
3. **Network issues**: Verify Ethernet/USB connection and configuration
4. **High temperature**: Check thermal solution and cooling

### Recovery Mode
1. To enter recovery mode, hold REC button while connecting power
2. Jetson can be reflashed using SDK Manager or command line tools

## Security Considerations

### Network Security
- Use static IP addresses for predictable communication
- Configure firewall for robot communication ports
- Use VPN for remote access to robot

### System Security
- Regularly update the system
- Disable unnecessary services
- Implement physical security measures

## Maintenance

### Regular Updates
1. Update system packages monthly:
```bash
sudo apt update && sudo apt upgrade -y
```

2. Update jetson-stats:
```bash
sudo -H pip3 install -U jetson-stats
```

### Performance Monitoring
1. Regularly monitor thermal performance with `jtop`
2. Check storage space and clean up as needed
3. Monitor network connection stability

## Integration with Robotic Platforms

### Connecting to Robot Hardware
1. Connect motor controllers via GPIO/SPI/I2C
2. Connect sensors (IMU, encoders, etc.)
3. Configure real-time performance if required

### Communication Protocols
1. Set up ROS 2 communication with main robot computer
2. Implement feedback protocols for control systems
3. Configure safety monitoring systems