# Data Model: Physical AI & Humanoid Robotics Course

## Core Entities

### Course Specification
- **Description**: The main deliverable document containing the 13-week module breakdown and technical requirements
- **Properties**:
  - id: string (unique identifier)
  - title: string
  - duration: number (in weeks)
  - learning_outcomes: array of strings
  - technical_requirements: object
  - hardware_requirements: object
  - capstone_project: object
- **Relationships**: Contains multiple Modules

### Module
- **Description**: A weekly period of the course with focused lessons and practical labs
- **Properties**:
  - id: string (unique identifier)
  - week_number: number
  - title: string
  - description: string
  - learning_objectives: array of strings
  - lessons: array of Lesson objects
  - labs: array of Lab objects
  - deliverables: array of Deliverable objects
- **Relationships**: Belongs to one Course Specification, contains multiple Lessons, Labs, and Deliverables

### Lesson
- **Description**: Individual theoretical component of a module
- **Properties**:
  - id: string (unique identifier)
  - title: string
  - content: string (Markdown format)
  - duration_minutes: number
  - prerequisites: array of strings (lesson IDs)
  - related_labs: array of strings (lab IDs)
- **Relationships**: Belongs to one Module

### Lab
- **Description**: Practical component with hands-on activities for the module
- **Properties**:
  - id: string (unique identifier)
  - title: string
  - description: string
  - steps: array of strings
  - required_equipment: array of strings
  - estimated_completion_time: number (in minutes)
  - learning_objectives: array of strings
  - related_lessons: array of strings (lesson IDs)
- **Relationships**: Belongs to one Module

### Deliverable
- **Description**: Assignment or project component that students complete for the module
- **Properties**:
  - id: string (unique identifier)
  - title: string
  - description: string
  - requirements: array of strings
  - submission_format: string
  - evaluation_criteria: array of strings
  - due_date: string (ISO date format)
- **Relationships**: Belongs to one Module

### Hardware Architecture
- **Description**: The specified components needed for the robotics lab
- **Properties**:
  - id: string (unique identifier)
  - description: string
  - workstation_spec: object (CPU, GPU, RAM, etc.)
  - robot_platforms: array of RobotPlatform objects
  - sensors: array of Sensor objects
  - edge_devices: array of EdgeDevice objects
  - cloud_infrastructure: object
- **Relationships**: Associated with Course Specification

### RobotPlatform
- **Description**: Physical robot used in the course
- **Properties**:
  - id: string (unique identifier)
  - name: string
  - model: string
  - manufacturer: string
  - specifications: object (DoF, payload, etc.)
  - ROS_compatibility: string
  - sensors: array of Sensor objects
  - actuators: array of Actuator objects
- **Relationships**: Belongs to one Hardware Architecture

### Sensor
- **Description**: Sensor component used on robots or in simulation
- **Properties**:
  - id: string (unique identifier)
  - name: string
  - type: string (LiDAR, camera, IMU, etc.)
  - model: string
  - specifications: object (accuracy, range, data rate, etc.)
  - interface: string (USB, Ethernet, etc.)
  - mounting_position: string (for robots)
- **Relationships**: Belongs to Hardware Architecture or Robot Platform

### Actuator
- **Description**: Actuator component used on robots
- **Properties**:
  - id: string (unique identifier)
  - name: string
  - type: string (servo, motor, etc.)
  - model: string
  - specifications: object (torque, speed, etc.)
  - interface: string (CAN, PWM, etc.)
  - mounting_position: string
- **Relationships**: Belongs to Robot Platform

### EdgeDevice
- **Description**: Edge computing hardware used on robots
- **Properties**:
  - id: string (unique identifier)
  - name: string
  - model: string
  - manufacturer: string
  - specifications: object (CPU, GPU, RAM, etc.)
  - OS: string
  - ROS_compatibility: string
  - power_requirements: object
- **Relationships**: Belongs to Hardware Architecture

### Capstone Project
- **Description**: The culminating project demonstrating multiple competencies
- **Properties**:
  - id: string (unique identifier)
  - title: string
  - description: string
  - workflow: object (Voice → Plan → Navigate → Perceive → Manipulate)
  - requirements: array of strings
  - evaluation_criteria: array of strings
  - timeline: object (start_date, milestones, end_date)
  - technical_components: array of strings
- **Relationships**: Belongs to Course Specification

### Docusaurus Documentation
- **Description**: Structured documentation format for the course
- **Properties**:
  - id: string (unique identifier)
  - title: string
  - type: string (tutorial, reference, guide, etc.)
  - content: string (Markdown format)
  - sidebar_position: number
  - related_topics: array of strings
  - prerequisites: array of strings
- **Relationships**: Associated with one or more Modules

## Validation Rules

### Course Specification Validation
- Duration must be exactly 13 weeks
- Must contain 13 modules
- Technical requirements must be compatible with hardware specifications
- Must include capstone project specification

### Module Validation
- Week_number must be unique within course
- Must have at least one lesson and one lab
- Learning objectives must align with course learning outcomes
- Duration of lessons and labs must fit within standard semester constraints

### Hardware Architecture Validation
- All required sensors must be compatible with specified robot platforms
- Workstation specifications must meet minimum requirements for simulation
- Edge devices must support real-time processing requirements

### Capstone Project Validation
- Must integrate components from multiple modules
- Workflow must follow Voice → Plan → Navigate → Perceive → Manipulate sequence
- Requirements must be achievable with specified hardware