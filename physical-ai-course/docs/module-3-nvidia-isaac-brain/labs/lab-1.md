# Lab 3.1: Setting Up Isaac Sim with Robot Model

## Overview
This lab provides hands-on experience with NVIDIA Isaac Sim by creating a simulation environment with a robot model. You'll learn to import robot models, configure sensors, and interface with the simulation using ROS 2.

## Prerequisites
- NVIDIA GPU with compute capability 6.0+
- Isaac Sim installed (Docker or native)
- ROS 2 Humble Hawksbill installed
- Basic understanding of Isaac Sim concepts from Lesson 3.1

## Learning Objectives
By the end of this lab, students will be able to:
- Launch Isaac Sim with GPU acceleration
- Import a robot model into Isaac Sim
- Configure sensors in Isaac Sim
- Interface Isaac Sim with ROS 2
- Control the robot and process sensor data

## Lab Duration
Estimated time: 4-5 hours

## Step-by-Step Instructions

### Step 1: Verify Isaac Sim Installation
1. If using Docker, ensure Docker is running and you have NVIDIA GPU access:
```bash
nvidia-smi
docker --version
```

2. Launch Isaac Sim container:
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
  nvcr.io/nvidia/isaac-sim:2023.1.0-hotfix1
```

3. For native installation, launch Isaac Sim directly:
```bash
./isaac-sim.sh
```

### Step 2: Create a New Stage
1. Open Isaac Sim
2. Create a new stage: File → New Stage
3. Save the stage in your projects folder: File → Save As → `isaac_ai_course/lesson1.usd`

### Step 3: Add Basic Environment Elements
1. In the Content window, navigate to `Isaac/Environments`
2. Drag and drop `Room.usd` into the viewport
3. Alternatively, create a simple environment manually:
   - Create a ground plane
   - Add some obstacles (cubes or cylinders)
   - Adjust lighting settings

### Step 4: Import a Robot Model
1. Isaac Sim includes several robot models in the Content browser:
   - Navigate to `Isaac/Robots`
   - Select `Franka/urdf/panda_alt_franka_hand.urdf` or another robot
   - Drag it into the stage

2. If you want to import your own robot:
   - Go to the Isaac ROS Warehouse
   - Import from a URDF or USD file
   - Set up the robot's articulation and drives

### Step 5: Configure Physics and Materials
1. Select the ground plane and set its physics properties:
   - Under Physics, ensure "Collides With" is enabled
   - Set appropriate friction coefficients

2. For your robot:
   - Verify that all links have proper collision geometries
   - Check that joint limits and ranges are correct
   - Assign appropriate materials for visual rendering

### Step 6: Add Sensors
1. In the Isaac Sim toolbar, find the "Create" menu
2. Add a RGB camera to your robot:
   - Right-click on the robot's head/base link
   - Add → Isaac Sensor → Isaac RGB Camera
   - Configure camera parameters (resolution, field of view, etc.)

3. Add a LIDAR sensor:
   - Right-click on the robot's main body
   - Add → Isaac Sensor → Isaac LIDAR
   - Configure LIDAR parameters (range, resolution, FOV)

### Step 7: Configure ROS 2 Bridge
1. Add the ROS 2 bridge extension:
   - Window → Extensions → Isaac ROS Extensions
   - Enable the ROS Bridge extension

2. Create a ROS 2 Context prim in your stage:
   - Create → Isaac → ROS2 → ROS2 Context

3. Create ROS 2 bridge publisher/subscriber nodes:
   - For camera: Create → Isaac → ROS2 → ROS2 Camera Helper
   - For LIDAR: Create → Isaac → ROS2 → ROS2 LIDAR Helper
   - For robot control: Create → Isaac → ROS2 → ROS2 Articulation Controller

### Step 8: Set Up ROS 2 Control
1. For robot control, you'll need:
   - Joint state publisher
   - Robot state publisher
   - Velocity or position controllers

2. Set up ROS 2 publisher/subscriber for your robot's joints:
   - Create ROS2 Control Bridge for your robot
   - Connect to appropriate joint drives

### Step 9: Configure the Simulation
1. In the Isaac Sim toolbar, go to "Window" → "Compute Graph"
2. Create a new compute graph to process sensor data
3. Add nodes for:
   - RGB camera to ROS 2 publisher
   - LIDAR to ROS 2 publisher
   - Joint states to ROS 2 publisher

### Step 10: Launch ROS 2 Bridge
1. If working outside Isaac Sim, launch the ROS 2 bridge:
```bash
source /opt/ros/humble/setup.bash
ros2 run rosbridge_server rosbridge_websocket
```

2. Or, if using Isaac ROS bridge:
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Recommended for Isaac Sim
ros2 launch isaac_ros_common rosbridge_launch.py
```

### Step 11: Test the Setup
1. Press the "Play" button in Isaac Sim
2. In a separate terminal, verify ROS 2 topics are being published:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

3. Verify you can see sensor data:
```bash
# Check camera data
ros2 topic echo /rgb_camera/image_raw

# Check LIDAR data
ros2 topic echo /lidar_scan

# Check joint states
ros2 topic echo /joint_states
```

### Step 12: Control the Robot
1. Publish velocity commands to move the robot:
```bash
# For a differential drive robot
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}' -1
```

2. Or for articulated robots, send joint commands:
```bash
# Example for a manipulator robot
ros2 topic pub /joint_group_position_controller/commands std_msgs/Float64MultiArray '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Step 13: Advanced Configuration (Optional)
1. Set up Isaac Sim's perception tools:
   - Add synthetic data generation components
   - Configure domain randomization
   - Set up ground truth annotations

2. Integrate with Isaac ROS packages:
   - Install relevant Isaac ROS packages
   - Use specialized perception nodes
   - Implement SLAM capabilities

### Step 14: Experiment and Extend
1. Modify the environment to create navigation challenges
2. Adjust sensor parameters to see how they affect data quality
3. Try different robot models
4. Implement a simple navigation algorithm that uses sensor data

## Deliverable
Submit a report containing:
1. Screenshots of your Isaac Sim environment with robot and sensors
2. ROS 2 topic listing showing active sensor publishers
3. Code that controls the robot based on sensor input
4. Video recording of the robot navigating in the environment
5. Analysis of sensor data quality and performance
6. Any challenges you encountered and how you resolved them

## Assessment Criteria
- Isaac Sim environment configured correctly (25%)
- Robot successfully imported and controllable (20%)
- Sensors properly configured and publishing data (25%)
- ROS 2 integration working properly (15%)
- Report is comprehensive and well-written (15%)

## Troubleshooting
- **GPU errors**: Ensure proper NVIDIA driver and CUDA installation
- **ROS connection issues**: Verify RMW implementation and network settings
- **Physics problems**: Check collision meshes and joint configurations
- **Sensor issues**: Verify sensor placement and parameters

## Additional Resources
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_basic.html)
- [Isaac ROS Tutorials](https://nvidia-isaac-ros.github.io/repositories_and_packages/index.html)
- [Omniverse USD Guide](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ros_bridge/docs/index.html)