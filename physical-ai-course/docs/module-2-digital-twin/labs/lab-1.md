# Lab 2.1: Creating Your First Gazebo Simulation Environment

## Overview
This lab provides hands-on experience with Gazebo simulation by creating a simple environment with a robot model. You'll learn to create world files, spawn robots, and interact with the simulation using ROS 2.

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Gazebo installed (gazebo-ros2-pkgs)
- Basic understanding of URDF from Module 1

## Learning Objectives
By the end of this lab, students will be able to:
- Create a Gazebo world file
- Spawn a robot model in Gazebo
- Control the robot using ROS 2 messages
- Use Gazebo plugins for sensors
- Launch simulation with ROS 2 launch files

## Lab Duration
Estimated time: 3-4 hours

## Step-by-Step Instructions

### Step 1: Verify Gazebo Installation
1. Check if Gazebo is installed:
```bash
gazebo --version
```

2. Install Gazebo ROS packages if not already installed:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

3. Test basic Gazebo functionality:
```bash
gazebo
```

### Step 2: Create a Basic World File
1. Create a directory for your worlds:
```bash
mkdir -p ~/physical_ai_ws/src/physical_ai_examples/worlds
```

2. Create a simple world file at `~/physical_ai_ws/src/physical_ai_examples/worlds/basic_room.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_room">
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Basic room environment -->
    <model name="room_walls">
      <!-- Front wall -->
      <pose>0 5 1 0 0 0</pose>
      <link name="front_wall">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Left wall -->
    <model name="left_wall">
      <pose>-5 0 1 0 0 1.5707</pose>
      <link name="left_wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Right wall -->
    <model name="right_wall">
      <pose>5 0 1 0 0 -1.5707</pose>
      <link name="right_wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Back wall -->
    <model name="back_wall">
      <pose>0 -5 1 0 0 3.1415</pose>
      <link name="back_wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a simple object in the center -->
    <model name="table">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="table_base">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Step 3: Create a Simple Robot Model
1. Create a directory for your models:
```bash
mkdir -p ~/physical_ai_ws/src/physical_ai_examples/models/my_robot
```

2. Create a model.config file at `~/physical_ai_ws/src/physical_ai_examples/models/my_robot/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>my_robot</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>

  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>

  <description>
    A simple robot model for the Physical AI course
  </description>
</model>
```

3. Create the SDF model file at `~/physical_ai_ws/src/physical_ai_examples/models/my_robot/model.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="my_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.1</iyy>
          <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.0 0.5 1.0 1</ambient>
          <diffuse>0.0 0.7 1.0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>

    <!-- Left wheel -->
    <link name="left_wheel">
      <pose>0 0.2 0 0 1.5707 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.4 0.4 0.4 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>

    <!-- Right wheel -->
    <link name="right_wheel">
      <pose>0 -0.2 0 0 1.5707 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.4 0.4 0.4 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>

    <!-- Joint connections -->
    <joint name="left_wheel_joint" type="continuous">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="right_wheel_joint" type="continuous">
      <parent>chassis</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <!-- Gazebo plugin for ROS2 control -->
    <gazebo reference="chassis">
      <material>Gazebo/Blue</material>
    </gazebo>
    
    <gazebo reference="left_wheel">
      <material>Gazebo/Black</material>
    </gazebo>
    
    <gazebo reference="right_wheel">
      <material>Gazebo/Black</material>
    </gazebo>

    <gazebo>
      <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.3</wheel_separation>
        <wheel_diameter>0.2</wheel_diameter>
        <max_wheel_torque>20</max_wheel_torque>
        <max_wheel_acceleration>1.0</max_wheel_acceleration>
        <command_topic>cmd_vel</command_topic>
        <odometry_topic>odom</odometry_topic>
        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>chassis</robot_base_frame>
        <publish_odom>true</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
        <publish_wheel_tf>true</publish_wheel_tf>
      </plugin>
    </gazebo>
  </model>
</sdf>
```

### Step 4: Create a Launch File
1. Create a launch directory:
```bash
mkdir -p ~/physical_ai_ws/src/physical_ai_examples/launch
```

2. Create a launch file at `~/physical_ai_ws/src/physical_ai_examples/launch/gazebo_sim.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('physical_ai_examples'),
            'worlds',
            'basic_room.world'
        ]),
        description='SDF world file'
    )

    # Launch Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Launch Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', PathJoinSubstitution([
                FindPackageShare('physical_ai_examples'),
                'models',
                'my_robot',
                'model.sdf'
            ]),
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world_cmd)

    # Add nodes and launch files
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
```

### Step 5: Build and Run
1. Make sure your package.xml includes the necessary dependencies:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>physical_ai_examples</name>
  <version>0.0.0</version>
  <description>Examples for Physical AI course</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>TODO: License declaration</license>

  <depend>gazebo_ros_pkgs</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>gazebo_dev</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

2. Build your workspace:
```bash
cd ~/physical_ai_ws
colcon build --packages-select physical_ai_examples
```

3. Source your workspace:
```bash
source install/setup.bash
```

4. Launch the simulation:
```bash
ros2 launch physical_ai_examples gazebo_sim.launch.py
```

### Step 6: Control the Robot
1. In a new terminal (after sourcing the workspace), send velocity commands:
```bash
# Move forward at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn in place at 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

2. Check the robot's odometry:
```bash
ros2 topic echo /odom
```

### Step 7: Experiment and Extend
1. Modify the world file to add more objects
2. Improve the robot model with additional sensors
3. Create a Python script that controls the robot to navigate around the table
4. Try different physics parameters for the robot

## Deliverable
Submit a report containing:
1. Screenshots of your Gazebo simulation with the robot in the environment
2. Modified code that demonstrates at least one extension from Step 7
3. Video recording of the robot moving in the environment
4. Explanation of physics parameters you chose and why
5. Any challenges you encountered and how you resolved them

## Assessment Criteria
- Simulation environment works correctly (30%)
- Robot spawns and moves as expected (25%)
- Code modifications are meaningful and well-documented (25%)
- Report is comprehensive and well-written (20%)

## Additional Resources
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [ROS 2 with Gazebo](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [SDF Model Format](http://sdformat.org/spec)