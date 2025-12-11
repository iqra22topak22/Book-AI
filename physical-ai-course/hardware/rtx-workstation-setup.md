# RTX Workstation Setup Guide

## Overview
This guide provides instructions for setting up RTX workstations for the Physical AI & Humanoid Robotics course. These workstations are essential for running simulations, training models, and processing data with high performance.

## Hardware Requirements

### Minimum Specifications
- **GPU**: NVIDIA RTX 3060 (12GB VRAM) or equivalent
- **CPU**: 8+ core processor (AMD Ryzen 7 or Intel i7)
- **RAM**: 32GB DDR4/DDR5
- **Storage**: 1TB SSD for OS and applications
- **OS**: Ubuntu 22.04 LTS

### Recommended Specifications
- **GPU**: NVIDIA RTX 4090 (24GB VRAM) or equivalent
- **CPU**: 16+ core processor (AMD Ryzen 9 or Intel i9)
- **RAM**: 64GB+ DDR4/DDR5
- **Storage**: 2TB+ NVMe SSD
- **OS**: Ubuntu 22.04 LTS
- **Network**: Gigabit Ethernet or better

## Software Installation

### 1. Operating System Installation
1. Download Ubuntu 22.04 LTS ISO from [ubuntu.com](https://ubuntu.com/download/desktop)
2. Create bootable USB drive using Rufus (Windows) or `dd` command (Linux)
3. Boot from USB and install Ubuntu with default settings
4. Update system packages:
```bash
sudo apt update && sudo apt upgrade -y
```

### 2. NVIDIA GPU Drivers
1. Check GPU model:
```bash
lspci | grep -E "VGA|3D"
```

2. Install NVIDIA drivers:
```bash
sudo apt update
sudo apt install nvidia-driver-535 nvidia-utils-535
```

3. Reboot the system
4. Verify installation:
```bash
nvidia-smi
```

### 3. CUDA Toolkit Installation
1. Download CUDA toolkit from [NVIDIA Developer](https://developer.nvidia.com/cuda-downloads)
2. Install CUDA toolkit:
```bash
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run
sudo sh cuda_12.3.0_545.23.06_linux.run
```

3. Update environment variables:
```bash
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

4. Verify installation:
```bash
nvcc --version
```

### 4. Docker Installation
1. Install Docker:
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```

2. Install NVIDIA Container Toolkit:
```bash
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

3. Test Docker with GPU:
```bash
docker run --rm --gpus all nvidia/cuda:12.3.0-base-ubuntu22.04 nvidia-smi
```

### 5. ROS 2 Installation
1. Add ROS 2 repository:
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US.UTF-8
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-nav2-bringup
```

2. Setup ROS 2 environment:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. Python Environment Setup
1. Install Python packages:
```bash
sudo apt install python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool
pip3 install numpy scipy torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
pip3 install openai-whisper
```

### 7. Isaac Sim Setup
1. Pull Isaac Sim Docker container:
```bash
docker pull nvcr.io/nvidia/isaac-sim:4.0.0
```

2. Create directory structure for Isaac Sim:
```bash
mkdir -p $HOME/isaac-sim/projects
mkdir -p $HOME/isaac-sim/cache/isaac-sim
mkdir -p $HOME/isaac-sim/cache/omni
```

### 8. Gazebo Setup
1. Install Gazebo Garden (recommended for ROS 2 Humble):
```bash
curl -sS https://get.gazebo.sim/gazebo.sh | bash
```

## Performance Optimization

### GPU Power Management
1. Set GPU to maximum performance mode:
```bash
sudo nvidia-smi -ac 10000,1800  # Adjust based on your GPU model
```

### System Optimizations
1. Disable unnecessary services:
```bash
sudo systemctl disable bluetooth
sudo systemctl disable cups
```

2. Configure swap for memory-intensive tasks:
```bash
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## Testing and Validation

### 1. GPU Compute Test
1. Run a simple CUDA program:
```bash
cd /usr/local/cuda/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery
```

### 2. Isaac Sim Test
1. Run Isaac Sim in Docker:
```bash
docker run --gpus all -it --rm --network=host \
  --env "ACCEPT_EULA=Y" \
  --env "USD_CACHEDIR=/isaac-sim/cache/isaac-sim/USD_CACHEDIR" \
  --env "OMNIVERSE_CACHEDIR=/isaac-sim/cache/omni/OMNIVERSE_CACHEDIR" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume $HOME/isaac-sim/cache/isaac-sim:/isaac-sim/cache/isaac-sim:rw \
  --volume $HOME/isaac-sim/cache/omni:/isaac-sim/cache/omni:rw \
  --volume $HOME/isaac-sim/projects:/isaac-sim/projects:rw \
  --env "DISPLAY=:0" \
  --privileged \
  --pid=host \
  nvcr.io/nvidia/isaac-sim:4.0.0
```

### 3. ROS 2 Test
1. Verify ROS 2 installation:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

## Troubleshooting

### Common Issues
1. **NVIDIA Driver Error**: Reinstall drivers using:
```bash
sudo apt purge nvidia-*  
sudo apt autoremove
sudo apt install nvidia-driver-535
```

2. **CUDA Not Working**: Verify installation path and environment variables:
```bash
echo $CUDA_HOME
which nvcc
```

3. **Docker GPU Access**: Ensure user is in docker group:
```bash
groups $USER
# If not in docker group: sudo usermod -aG docker $USER
```

## Maintenance

### Regular Updates
1. Update NVIDIA drivers monthly
2. Update CUDA toolkit as needed
3. Update ROS 2 packages regularly:
```bash
sudo apt update && sudo apt upgrade
```

### Storage Cleanup
1. Clean Docker images periodically:
```bash
docker system prune -a
```

2. Clear Isaac Sim caches:
```bash
rm -rf $HOME/isaac-sim/cache/*
```

## Security Considerations

### Network Security
- Restrict network access to simulation environments
- Use VPN for remote access to workstations
- Implement firewall rules for service ports

### Data Security
- Encrypt sensitive simulation data
- Regularly backup configuration files
- Use secure authentication for remote access