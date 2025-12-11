# Unitree Robot Integration Guide

## Overview
This guide provides instructions for integrating Unitree robot platforms (Go2, G1) with the Physical AI & Humanoid Robotics course. Unitree robots are advanced quadruped and humanoid platforms with ROS 2 compatibility and extensive documentation.

## Supported Unitree Models

### Unitree Go2
- **Type**: Quadruped robot
- **Leg Count**: 4 legs with 3 degrees of freedom each
- **Height**: 0.5 m (standing), 0.35 m (squatting)
- **Weight**: 12 kg
- **Payload**: 3 kg
- **Battery**: 3.6Ah, 25.2V Li-ion (2+ hours operation time)
- **Sensors**: 
  - Depth camera (optional)
  - IMU
  - Force/torque sensors in legs
- **Communication**: Wi-Fi, Ethernet, USB
- **SDK**: Python, C++, ROS 2 support

### Unitree G1
- **Type**: Humanoid robot
- **Height**: 1.15 m
- **Weight**: 32 kg (excluding battery)
- **Degrees of Freedom**: 24 motors
- **Battery**: 26.4Ah, 30.6V Li-ion battery (1+ hours operation time)
- **Sensors**: 
  - RGB-D camera
  - IMU
  - Force/torque sensors in feet
  - Joint position sensors
- **Communication**: 5G, Wi-Fi, Ethernet, USB
- **SDK**: Python, C++, ROS 2 support

## Prerequisites

### Hardware Requirements
- Unitree robot platform (Go2 or G1)
- Router for Wi-Fi connection or wired Ethernet
- Computer with Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed

### Network Setup
- Dedicated network for robot communication
- Static IP assignment recommended
- Firewall rules allowing communication on required ports

## Hardware Setup

### 1. Physical Inspection
1. Inspect robot for shipping damage
2. Charge batteries to full capacity before first use
3. Verify all limbs/motors are securely attached
4. Check camera and sensor positioning (for models with camera options)

### 2. Power On Sequence
1. Turn on robot: Hold power button for 3 seconds
2. Wait for system initialization (approx. 30 seconds)
3. LED indicators will show operational status
4. Verify all joints have minimal play/tightness

### 3. Network Configuration
1. For Go2:
   - Robot creates its own Wi-Fi network: `unitree-go2-XXXX`
   - Connect to robot's network to configure
   - Or configure to join existing network

2. For G1:
   - Uses 5G/Wi-Fi for remote communication
   - Also supports Ethernet connection
   - Configure network via unitree remote interface

## Software Installation

### 1. Unitree SDK Installation
1. Register on Unitree developer portal to download SDK
2. Install dependencies:
```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install -y build-essential cmake git
```

2. Download and install Unitree ROS 2 Package:
```bash
# Create workspace
mkdir -p ~/unitree_ws/src
cd ~/unitree_ws/src

# Clone Unitree ROS 2 packages
git clone https://github.com/unitreerobotics/unitree_ros2.git
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
git clone https://github.com/unitreerobotics/yaml_cpp_catkin.git
```

### 2. ROS 2 Dependencies
1. Install additional ROS 2 packages:
```bash
sudo apt install -y ros-humble-xacro ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard
```

### 3. Build the Workspace
```bash
cd ~/unitree_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select unitree_ros2 yaml_cpp_catkin
```

### 4. Environment Setup
1. Add to your `.bashrc`:
```bash
echo "source ~/unitree_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Network Configuration

### 1. Robot Network Setup
1. For Wi-Fi configuration (Go2):
```bash
# Connect PC to robot's AP: unitree-go2-XXXX
# Access robot via web interface
# Configure to connect to your network
```

2. For static IP assignment:
```bash
# On robot side (if configurable)
# Configure static IP in 192.168.x.x range
# Common robot IPs: 192.168.123.164 for Go2, 192.168.123.161 for G1
```

### 2. Firewall Configuration
1. Allow required ports:
```bash
# Unitree communication typically uses UDP ports
sudo ufw allow 8080
sudo ufw allow 8003:8006/udp
```

## ROS 2 Integration

### 1. Launch Basic Control
1. Bring up robot communication:
```bash
# Make sure robot is powered on and connected
source ~/unitree_ws/install/setup.bash
source /opt/ros/humble/setup.bash

# For Go2 teleop
ros2 launch unitree_ros2 go2.launch.py

# For G1 teleop
ros2 launch unitree_ros2 g1.launch.py
```

### 2. Basic Movement Commands
1. Test basic movements:
```bash
# In a separate terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Robot State Monitoring
1. Monitor robot state:
```bash
# Joint states
ros2 topic echo /joint_states

# IMU data
ros2 topic echo /trunk_imu

# Battery status
ros2 topic echo /battery_state
```

## Unitree SDK Direct Integration

### 1. C++ SDK Example
1. Create basic communication node:

```cpp
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): ctrlComp(LOWLEVEL), udp(level) {
        ctrlComp.addLayer(&lowCmd);
        ctrlComp.addLayer(&lowState);

        udp.InitCmdData(cmd);
        udp.InitRecvData(state);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    CtrlComponent ctrlComp;
    LowCmd lowCmd;
    LowState lowState;
    UDPPort udp;
    LowHighLevelCtrlCmd cmd;
    LowHighLevelCtrlState state;
};

void Custom::UDPRecv(){
    udp.Recv();
}

void Custom::UDPSend(){
    udp.Send();
}

void Custom::RobotControl(){ 
    lowCmd.levelFlag = LOWLEVEL;
    
    for(int i = 0; i < 20; i++){
        lowCmd.motorCmd[i].q = state.motorState[i].q;
        lowCmd.motorCmd[i].dq = 0;
        lowCmd.motorCmd[i].tau = 0;
        lowCmd.motorCmd[i].Kp = 0;
        lowCmd.motorCmd[i].Kd = 0;
    }

    // Example: Moving one leg
    // lowCmd.motorCmd[0].q = 0.01; // slight adjustment to first joint
}

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl;
    std::cout << "WARNING: Make sure the robot is hung up." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
    LoopFunc loop_control("control_loop", 0.002, 3, &custom, &Custom::RobotControl);  // 500Hz
    LoopFunc loop_udpSend("udp_send", 0.002, 3, &custom, &Custom::UDPSend);             // 500Hz
    LoopFunc loop_udpRecv("udp_recv", 0.002, 3, &custom, &Custom::UDPRecv);             // 500Hz

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    }

    return 0;
}
```

### 2. Python SDK Example
1. Create basic Python controller:

```python
import sys
import time
import math
import numpy as np

# Install the unitree SDK Python package
# pip3 install unitree-sdk

def example_go2():
    # Example for Go2
    from unitree_sdk import *
    
    # Initialize the robot
    robot = Robot("go2")
    
    # Connect to robot (replace with your robot's IP)
    robot.connect("192.168.123.164")
    
    # Set robot to a position where it's standing
    robot.stand_up()
    
    # Wait a moment
    time.sleep(2)
    
    # Move forward
    robot.move_forward(0.5, duration=3.0)  # Move forward 0.5 at 0.5m/s for 3 seconds
    
    # Sit down
    robot.sit_down()
    
    # Disconnect
    robot.disconnect()

if __name__ == "__main__":
    example_go2()
```

## URDF and Visualization

### 1. Robot Description
1. Install robot description packages:
```bash
cd ~/unitree_ws/src
git clone https://github.com/unitreerobotics/unitree_ros_description.git
cd ~/unitree_ws
source install/setup.bash
colcon build --packages-select unitree_ros_description
```

### 2. Visualize Robot in RViz
1. Launch robot model:
```bash
source ~/unitree_ws/install/setup.bash
ros2 launch unitree_ros_description go2.launch.py  # or g1.launch.py
```

2. In another terminal launch RViz:
```bash
rviz2
# Add RobotModel display and set topic to /robot_description
```

## Control Modes

### 1. Position Control
- Direct joint position control
- Useful for precise positioning tasks
- Requires careful trajectory planning

### 2. Velocity Control
- Joint velocity control
- Good for coordinated movements
- Smoother transitions than position control

### 3. Impedance Control
- Variable stiffness/impedance control
- Safe for contact interactions
- Important for manipulation tasks

### 4. High-Level Motion Control
- Pre-programmed gaits and motions
- Walking, trotting, dancing, etc.
- Provided by Unitree SDK

## Safety Considerations

### 1. Physical Safety
- Always ensure robot is properly secured during initial testing
- Maintain safe distances during high-speed movements
- Keep clear area when testing locomotion patterns
- Have emergency stop procedure ready

### 2. Electrical Safety
- Proper grounding during charging
- Avoid moisture exposure
- Use official chargers only
- Monitor battery temperature

### 3. Emergency Procedures
- Know how to power off robot in emergency
- Establish clear communication protocols
- Have fire extinguisher nearby when testing

## Calibration

### 1. IMU Calibration
1. Follow Unitree calibration procedures:
```bash
# Run the IMU calibration tool included in SDK
ros2 run unitree_ros2 imu_calibrator
```

### 2. Motor Calibration
1. Motors may need recalibration after transport:
```bash
# Follow manufacturer's calibration procedure
# This is usually done automatically during boot
```

## Troubleshooting

### Common Issues
1. **No Connection**: 
   - Verify network connectivity
   - Check firewall settings
   - Verify robot IP address

2. **Motor Errors**:
   - Check for obstructions
   - Verify motor temperatures
   - Check for error codes in logs

3. **Communication Drops**:
   - Ensure stable network connection
   - Check packet loss on network
   - Reduce network congestion

### Diagnostic Commands
```bash
# Check network connectivity
ping 192.168.123.164  # Replace with robot IP

# Monitor ROS topics
ros2 topic list
ros2 topic echo /trunk_imu

# Check robot status
ros2 service call /robot_status std_msgs/srv/Empty
```

## Integration with Course Modules

### 1. ROS 2 Module (Module 1)
- Use Unitree as example of ROS 2 integration
- Demonstrate multi-node communication
- Implement custom controllers

### 2. Digital Twin (Module 2)
- Create simulation of Unitree robot
- Validate control algorithms in simulation first
- Bridge simulation to real robot

### 3. NVIDIA Isaac (Module 3)
- Integrate with Isaac Sim models
- Use Isaac ROS perception packages
- Implement visual navigation

### 4. Voice-Language-Action (Module 4)
- Connect voice commands to robot actions
- Implement speech-activated control
- Create voice-interactive demonstrations

## Advanced Features

### 1. Perception Integration
- Connect RGB-D cameras to perception stack
- Implement SLAM with Unitree as mobile platform
- Use camera for manipulation tasks

### 2. Custom Gaits Development
- Design new locomotion patterns
- Optimize for specific terrain
- Implement adaptive gait switching

### 3. Autonomous Navigation
- Integrate with Nav2 for navigation
- Use robot's IMU and encoders
- Plan paths around obstacles

## Maintenance

### Regular Checks
1. Inspect legs and joints for wear
2. Check battery charge cycles and performance
3. Verify sensor calibration
4. Update robot firmware regularly
5. Clean robot after field tests

### Long Term Care
- Store upright with minimal pressure on joints
- Charge batteries to 40-60% for long-term storage
- Protect from moisture and dust
- Replace worn components promptly