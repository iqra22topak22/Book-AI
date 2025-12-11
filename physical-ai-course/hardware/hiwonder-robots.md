# Hiwonder Robot Integration Guide

## Overview
This guide provides instructions for integrating Hiwonder robot platforms with the Physical AI & Humanoid Robotics course. Hiwonder offers a variety of educational and research robotics platforms with good ROS compatibility and documentation.

## Supported Hiwonder Models

### Hiwonder Ranger EP (Educational Quadruped)
- **Type**: Educational quadruped robot
- **Leg Count**: 4 legs with 3 degrees of freedom per leg
- **Height**: Adjustable (typically 20-30cm)
- **Weight**: Approximately 1.5 kg
- **Controller**: STM32-based control board
- **Communication**: Wi-Fi, Bluetooth, serial
- **Programming**: Arduino IDE, Python, ROS support
- **Sensors**: IMU, ultrasonic sensor (depending on kit)

### Hiwonder SCOUT MINI (Mobile Manipulation Platform)
- **Type**: Ground mobile manipulator platform
- **Base**: Ackermann steering mobile platform
- **Arm**: 5-6 DOF robotic arm (varies by kit)
- **Controller**: Raspberry Pi 4 or Jetson Nano support
- **Sensors**: IMU, ultrasonic sensors, camera
- **Programming**: ROS, Python, C++

### Hiwonder ArmPi Pro (Desktop Manipulator)
- **Type**: Desktop 6-DOF robotic arm
- **DOF**: 6 joints + gripper
- **Controller**: ESP32-based controller
- **Actuators**: Servo motors (typically Dynamixel-compatible)
- **Sensors**: IMU, optional camera
- **Programming**: Python, ROS, Arduino IDE

## Prerequisites

### Hardware Requirements
- Hiwonder robot platform
- Router for Wi-Fi connection or computer for direct connection
- Computer with Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Compatible power adapters for the robot

### Software Requirements
- Arduino IDE for firmware programming
- Python 3.10+ for high-level control
- ROS 2 Humble with additional packages

## Hardware Setup

### 1. Physical Assembly
1. Verify all components are included per the kit documentation
2. Follow Hiwonder's assembly guide carefully
3. Secure all connections, especially servo motors
4. Verify proper mechanical alignment

### 2. Power Connections
1. Connect power to main controller board
2. Verify power distribution to servos and sensors
3. Check all voltage requirements match specifications
4. Connect battery or power adapter according to specifications

### 3. Communication Interface
1. Establish communication method:
   - **Wi-Fi**: Connect to robot's AP and control via network
   - **USB Serial**: Direct connection for low-latency control
   - **Bluetooth**: Short-range wireless control
2. Test basic communication with controller

## Software Installation

### 1. Hiwonder SDK Installation
1. Download Hiwonder software packages from their official site:
```bash
mkdir -p ~/hiwonder_ws/src
cd ~/hiwonder_ws/src

# Clone Hiwonder ROS packages
git clone https://github.com/hiwonder/hiwonder_ros.git
git clone https://github.com/hiwonder/hiwonder_sdk.git
```

### 2. Dependencies Installation
1. Install required dependencies:
```bash
sudo apt update
sudo apt install -y python3-pip python3-serial python3-opencv
pip3 install pyserial dynamixel-sdk cv2

# Install ROS 2 packages
sudo apt install -y ros-humble-xacro ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard ros-humble-image-pipeline
```

### 3. Build Hiwonder Packages
```bash
cd ~/hiwonder_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 4. Environment Setup
1. Add to your `.bashrc`:
```bash
echo "source ~/hiwonder_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## ROS 2 Integration

### 1. Launch File Configuration
1. Create launch files for your specific Hiwonder robot model:
```python
# Example for Hiwonder quadruped (ranger_ep)
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'hiwonder_ros'
    urdf_dir = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(urdf_dir, 'urdf', 'ranger_ep.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    params = {'robot_description': robot_desc}
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='hiwonder_ros',
            executable='quadruped_control_node',
            output='screen',
            parameters=[
                {'port_name': '/dev/ttyUSB0'},  # Adjust port as needed
                {'baud_rate': 1000000}
            ]
        )
    ])
```

### 2. Basic Communication Test
1. Launch the robot interface:
```bash
source ~/hiwonder_ws/install/setup.bash
ros2 launch hiwonder_ros ranger_ep.launch.py  # or appropriate launch file
```

2. Test basic commands:
```bash
# Test joint states
ros2 topic echo /joint_states

# Send basic movement commands if available
# The specific topics depend on the robot model
```

## Controller Integration

### 1. Joint Controllers
1. Hiwonder robots typically use servo motors that can be controlled:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class HiwonderController(Node):
    def __init__(self):
        super().__init__('hiwonder_controller')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        
        # Subscriber for joint states
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control
        
        self.joint_positions = {}
        
        self.get_logger().info("Hiwonder Controller initialized")
    
    def joint_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
    
    def control_loop(self):
        """Main control loop"""
        # Example: Send a simple movement command
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['leg_front_left_1', 'leg_front_left_2', 'leg_front_left_3']  # Example joint names
        msg.position = [0.0, 0.0, 0.0]  # Example positions
        
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = HiwonderController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Communication with Microcontroller
1. Hiwonder robots often use microcontrollers for low-level control:
```cpp
// Example Arduino code for Hiwonder controller
#include <Arduino.h>
#include <Servo.h>

#define NUM_SERVOS 12  // Adjust based on your robot

Servo servos[NUM_SERVOS];
int servo_pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};  // Adjust pins

void setup() {
    Serial.begin(1000000);  // Use appropriate baud rate
  
    // Attach all servos
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(servo_pins[i]);
    }
    
    // Initialize servos to neutral position
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(90);
    }
}

void loop() {
    // Check for incoming commands from ROS
    if (Serial.available()) {
        process_command();
    }
}

void process_command() {
    // Process ROS command (example: joint angles)
    String cmd = Serial.readStringUntil('\n');
    // Parse and execute the command
    // This is simplified - actual implementation depends on your robot
}
```

## Python API Integration

### 1. Direct Python Control
1. Create Python interface for direct robot control:
```python
import serial
import time
import struct

class HiwonderRobot:
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.connect()
    
    def connect(self):
        """Establish serial connection to robot"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for connection to stabilize
            self.get_logger().info(f"Connected to Hiwonder robot on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not connect to robot: {e}")
    
    def send_command(self, command_id, params):
        """Send command to robot"""
        if not self.serial_conn:
            return False
            
        # Construct packet (format depends on robot model)
        packet = self.construct_packet(command_id, params)
        
        try:
            self.serial_conn.write(packet)
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")
            return False
    
    def construct_packet(self, cmd_id, params):
        """Construct protocol packet based on Hiwonder protocol"""
        # This will vary by robot model
        # Example for a generic servo command:
        header = [0xFF, 0xFF]  # Packet header
        id_byte = [0xFE]      # Servo ID
        length = [len(params) + 3]  # Length including ID, LEN, CMD, CHKSUM
        cmd = [cmd_id]
        data = params
        checksum = 255 - ((sum(id_byte + length + cmd + data)) % 256)
        
        packet = header + id_byte + length + cmd + data + [checksum]
        return bytearray(packet)
    
    def move_servos(self, servo_positions):
        """Move multiple servos to specified positions"""
        # servo_positions: dict {servo_id: position}
        for servo_id, pos in servo_positions.items():
            # Convert position to servo command format
            pos_bytes = [pos & 0xFF, (pos >> 8) & 0xFF]
            self.send_command(0x03, [servo_id] + pos_bytes)  # Example command ID
    
    def disconnect(self):
        """Close the connection"""
        if self.serial_conn:
            self.serial_conn.close()


# Example usage:
if __name__ == "__main__":
    robot = HiwonderRobot()
    
    # Move servos to neutral position
    neutral_pos = {i: 512 for i in range(1, 13)}  # Example: 12 servos
    robot.move_servos(neutral_pos)
    
    time.sleep(2)
    
    # Disconnect
    robot.disconnect()
```

### 2. ROS 2 Node Integration
1. Create ROS 2 node that wraps the Python API:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from hiwonder_robot import HiwonderRobot  # Import your direct control API

class HiwonderRosBridge(Node):
    def __init__(self):
        super().__init__('hiwonder_ros_bridge')
        
        # Initialize robot connection
        self.robot = HiwonderRobot(port='/dev/ttyUSB0')
        
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber for joint commands
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray, 
            'joint_commands', 
            self.joint_cmd_callback, 
            10
        )
        
        # Timer for publishing joint states
        self.state_timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz
        
        self.get_logger().info("Hiwonder ROS Bridge initialized")
    
    def joint_cmd_callback(self, msg):
        """Handle joint command messages"""
        # Convert Float64MultiArray to servo positions dict
        servo_positions = {}
        for i, pos in enumerate(msg.data):
            # Convert position to your robot's servo units
            servo_positions[i+1] = int(pos * 1024 / 360) + 512  # Example conversion
        
        # Send to robot
        self.robot.move_servos(servo_positions)
    
    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Get current positions from robot (this would need to be implemented in your HiwonderRobot class)
        # For now, we'll use placeholders
        joint_names = [f'joint_{i}' for i in range(12)]  # Adjust based on your robot
        joint_positions = [0.0] * 12  # Placeholder values
        
        msg.name = joint_names
        msg.position = joint_positions
        
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HiwonderRosBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## URDF and Visualization

### 1. Robot Description Package
1. Create URDF for your Hiwonder robot:
```xml
<?xml version="1.0"?>
<robot name="ranger_ep" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Leg macros -->
  <xacro:property name="leg_length" value="0.05"/>
  <xacro:property name="leg_radius" value="0.01"/>

  <!-- Function to create a leg -->
  <xacro:macro name="leg" params="prefix x_reflect y_reflect">
    <joint name="${prefix}_hip_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_hip"/>
      <origin xyz="${x_reflect*0.07} ${y_reflect*0.05} 0" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="3.14"/>
    </joint>
    
    <link name="${prefix}_hip">
      <visual>
        <geometry>
          <cylinder length="${leg_length}" radius="${leg_radius}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
    </link>
    
    <joint name="${prefix}_thigh_joint" type="revolute">
      <parent link="${prefix}_hip"/>
      <child link="${prefix}_thigh"/>
      <origin xyz="0 0 -${leg_length/2}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="3.14"/>
    </joint>
    
    <link name="${prefix}_thigh">
      <visual>
        <geometry>
          <cylinder length="0.1" radius="${leg_radius}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
    </link>
    
    <joint name="${prefix}_calf_joint" type="revolute">
      <parent link="${prefix}_thigh"/>
      <child link="${prefix}_foot"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="3.14"/>
    </joint>
    
    <link name="${prefix}_foot">
      <visual>
        <geometry>
          <sphere radius="0.015"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
    </link>
  </xacro:macro>

  <!-- Create all four legs -->
  <xacro:leg prefix="front_left" x_reflect="1" y_reflect="1"/>
  <xacro:leg prefix="front_right" x_reflect="1" y_reflect="-1"/>
  <xacro:leg prefix="rear_left" x_reflect="-1" y_reflect="1"/>
  <xacro:leg prefix="rear_right" x_reflect="-1" y_reflect="-1"/>
</robot>
```

### 2. Launch for Visualization
1. Create launch file for RViz visualization:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf_dir = get_package_share_directory('hiwonder_ros')
    urdf_file = os.path.join(urdf_dir, 'urdf', 'ranger_ep.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    params = {'robot_description': robot_desc}
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[params]
        ) if LaunchConfiguration('use_rviz').perform({}) == 'true' else None
    ])
```

## Network Configuration

### 1. Wi-Fi Setup (if applicable)
1. If your Hiwonder robot supports Wi-Fi control:
```bash
# Connect to robot's network
# SSID format: typically hiwonder_* or similar
nmcli dev wifi connect "hiwonder_robot" password "password123"

# Or using traditional method:
sudo iwconfig wlan0 essid "hiwonder_robot"
sudo dhclient wlan0  # Obtain IP via DHCP
```

### 2. Serial Port Permissions
1. Add your user to dialout group for serial access:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

## Safety Considerations

### 1. Physical Safety
- Always supervise robot operation during initial tests
- Start with minimal power/movement settings
- Ensure safe operating environment
- Have emergency stop procedure ready

### 2. Electrical Safety
- Verify power supply voltages match specifications
- Monitor motor temperatures during extended operation
- Ensure proper grounding
- Use appropriate fuses/circuit protection

### 3. Communication Safety
- Verify command validation before sending to robot
- Implement bounds checking on joint positions
- Use emergency stop mechanisms

## Troubleshooting

### Common Issues
1. **No Communication**:
   - Check physical connections
   - Verify device permissions (`ls -l /dev/tty*`)
   - Confirm baud rate and protocol settings

2. **Joint Limit Errors**:
   - Check servo configuration
   - Verify physical assembly
   - Confirm homing procedures

3. **Control Instability**:
   - Adjust PID parameters
   - Check power supply adequacy
   - Verify sensor feedback

### Diagnostic Commands
```bash
# Check serial devices
ls -l /dev/tty*

# Test serial communication
echo -e \\x01 | hexdump -C

# Monitor ROS topics
ros2 topic list
ros2 topic echo /joint_states
```

## Integration with Course Modules

### 1. ROS 2 Module (Module 1)
- Use Hiwonder as practical example of ROS 2 messaging
- Implement custom controllers
- Demonstrate multi-package integration

### 2. Digital Twin (Module 2)
- Create simulation model of Hiwonder robot
- Implement simulation-to-reality transfer
- Validate control algorithms in both domains

### 3. NVIDIA Isaac (Module 3)
- Integrate perception with Hiwonder's sensors
- Implement navigation algorithms
- Use Isaac Sim for advanced simulation

### 4. Voice-Language-Action (Module 4)
- Connect voice commands to Hiwonder robot actions
- Implement natural language processing
- Create interactive demonstrations

## Advanced Features

### 1. Perception Integration
- Connect cameras to perception pipelines
- Implement object detection and tracking
- Use for autonomous navigation

### 2. Manipulation
- For arm-equipped models
- Implement grasp planning
- Control grippers and end-effectors

### 3. Multi-Robot Coordination
- Coordinate multiple Hiwonder robots
- Implement swarm behaviors
- Use for research projects

## Maintenance

### Regular Checks
1. Inspect servo mounts and linkages
2. Check battery performance and charging cycles
3. Verify sensor calibration
4. Update firmware when available
5. Clean dust from joints and electronics

### Long Term Care
- Store with joints at neutral positions
- Protect electronics from moisture
- Replace worn servo horns/tapes as needed
- Maintain backup firmware