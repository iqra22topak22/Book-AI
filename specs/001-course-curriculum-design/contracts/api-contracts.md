# Course Interface Contracts: Physical AI & Humanoid Robotics

## ROS 2 Node Interfaces

### Navigation Node Interface (`physical_ai_nav`)

#### Service: `/navigate_to_pose`
- **Request**: `geometry_msgs/PoseStamped`
  - Contains target pose (position and orientation) in map frame
- **Response**: `nav2_msgs/NavigationResult`
  - Contains navigation status (SUCCESS, FAILED, CANCELLED)
  - Execution time and path metrics

#### Topic: `/current_pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Frequency**: 10 Hz
- **Description**: Current robot pose in the map frame

#### Topic: `/cmd_vel`
- **Type**: `geometry_msgs/Twist`
- **Frequency**: 50 Hz (max)
- **Description**: Velocity commands for robot base

### Perception Node Interface (`physical_ai_perception`)

#### Service: `/detect_objects`
- **Request**: `sensor_msgs/Image`
  - Image from robot's RGB camera
- **Response**: `object_detection_msgs/Objects`
  - Array of detected objects with bounding boxes, class labels, and confidence scores

#### Topic: `/camera/rgb/image_raw`
- **Type**: `sensor_msgs/Image`
- **Frequency**: 15-30 Hz
- **Description**: Raw RGB camera feed

#### Topic: `/camera/depth/image_raw`
- **Type**: `sensor_msgs/Image`
- **Frequency**: 15-30 Hz
- **Description**: Depth image from RGB-D sensor

### Voice Command Node Interface (`voice_command`)

#### Service: `/process_speech`
- **Request**: `std_msgs/String`
  - Raw audio input or text input
- **Response**: `physical_ai_msgs/VoiceCommand`
  - Parsed command with intent and parameters

#### Topic: `/voice_input`
- **Type**: `audio_common_msgs/AudioData`
- **Frequency**: As available
- **Description**: Raw audio stream from microphone array

### Manipulation Node Interface (`physical_ai_manipulation`)

#### Service: `/pick_object`
- **Request**: `geometry_msgs/Pose`
  - Pose of object to be picked relative to robot base
- **Response**: `physical_ai_msgs/ManipulationResult`
  - Success/failure status and execution details

#### Service: `/place_object`
- **Request**: `geometry_msgs/Pose`
  - Target placement pose relative to robot base
- **Response**: `physical_ai_msgs/ManipulationResult`
  - Success/failure status and execution details

## Simulation Interfaces

### Gazebo/Unity Simulation Control

#### Service: `/reset_simulation`
- **Request**: Empty
- **Response**: Empty
- **Description**: Reset simulation world to initial state

#### Service: `/spawn_object`
- **Request**: `gazebo_msgs/SpawnModel`
  - Model definition and spawn pose
- **Response**: `gazebo_msgs/SpawnModelResponse`
  - Success status and model name

### Isaac Sim Interfaces

#### Service: `/get_robot_state`
- **Request**: Empty
- **Response**: `sensor_msgs/JointState`
  - Current joint positions, velocities, and efforts

#### Service: `/set_robot_state`
- **Request**: `sensor_msgs/JointState`
  - Target joint positions for robot actuators
- **Response**: Empty
  - Command accepted status

## Course Documentation API

### Module Content Service

#### REST Endpoint: `/api/v1/modules/{module_id}`
- **GET**: Retrieve module specification including lessons, labs, and deliverables
- **Response**: JSON object with module data as per data model

#### REST Endpoint: `/api/v1/modules/{module_id}/lab/{lab_id}/start`
- **POST**: Initialize lab environment (simulation, hardware, etc.)
- **Response**: Status of lab initialization with connection details

### Assessment Service

#### REST Endpoint: `/api/v1/assessments/submit`
- **POST**: Submit deliverable for grading
- **Request Body**: Submission package with code, results, and documentation
- **Response**: Grading results and feedback

#### REST Endpoint: `/api/v1/assessments/{deliverable_id}/evaluate`
- **POST**: Request evaluation of student submission
- **Response**: Detailed evaluation results and scoring

## Hardware Interface Contracts

### Jetson Orin Interface

#### Service: `/jetson/status`
- **Request**: Empty
- **Response**: `diagnostic_msgs/DiagnosticArray`
  - System health, temperature, load, and available resources

#### Topic: `/jetson/system_stats`
- **Type**: `diagnostic_msgs/DiagnosticArray`
- **Frequency**: 1 Hz
- **Description**: Continuous system monitoring data

### Robot Platform Interface

#### Service: `/robot/platform_info`
- **Request**: Empty
- **Response**: `physical_ai_msgs/RobotInfo`
  - Robot model, capabilities, sensor configuration

#### Service: `/robot/calibrate`
- **Request**: Empty or calibration parameters
- **Response**: Calibration results and status

## Capstone Project Workflow Interface

### Voice Command to Action Pipeline

#### Component: Voice-to-Intent Parser
- **Input**: `std_msgs/String` (voice command)
- **Output**: `physical_ai_msgs/Intent`
  - Action type, target, parameters

#### Component: Action Planner
- **Input**: `physical_ai_msgs/Intent`
- **Output**: `nav_msgs/Path` and action sequence
  - Navigation path and manipulation sequence

#### Component: Action Executor
- **Input**: Action sequence from planner
- **Output**: Execution status and feedback