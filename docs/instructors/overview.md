---
sidebar_position: 12
---

# Instructor Resources

## Overview
This guide provides resources for instructors teaching the Physical AI & Humanoid Robotics course. It includes curriculum guidance, assessment suggestions, technical setup instructions, and practical tips for effective course delivery.

## Course Structure Overview

### 13-Week Course Schedule
The course is structured as a 13-week program with the following module breakdown:

| Week | Module | Topics | Learning Outcomes |
|------|--------|--------|-------------------|
| 1-3 | Module 1: ROS 2 Nervous System | ROS 2 fundamentals, nodes, topics, services, URDF, rclpy bridge | Students can implement basic ROS 2 communication patterns |
| 4-6 | Module 2: Digital Twin | Gazebo, Unity, physics simulation, sensor modeling | Students can create simulation environments for robotics |
| 7-9 | Module 3: NVIDIA Isaac Brain | Isaac Sim, Isaac ROS, VSLAM, Nav2 | Students can implement navigation and perception with Isaac |
| 10-12 | Module 4: Vision-Language-Action | Whisper, LLM planning, VLA systems | Students can integrate voice, language, and action |
| 13 | Capstone Project | Integrates all modules | Students demonstrate complete VLA pipeline |

## Prerequisites for Students

### Technical Prerequisites
- Basic programming skills (Python preferred)
- Understanding of linear algebra and calculus
- Familiarity with Linux command line
- Basic understanding of robotics concepts (recommended)

### Hardware Requirements
- RTX workstation (minimum RTX 3060, recommended RTX 4090)
- Robot platform (Unitree Go2/G1 or Hiwonder equivalent)
- Sensors (RealSense, IMU, microphone array)
- Network infrastructure for robot communication

### Software Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA drivers and CUDA toolkit
- Isaac Sim (if available)
- Git version control

## Learning Objectives by Module

### Module 1 Learning Objectives
By the end of Module 1, students should be able to:
- Implement and run basic ROS 2 publisher/subscriber systems
- Create ROS 2 services and actions
- Define robot models using URDF
- Interface with hardware using rclpy

### Module 2 Learning Objectives
By the end of Module 2, students should be able to:
- Create simulation environments in Gazebo
- Model physics properties accurately
- Integrate sensors in simulation environments
- Validate simulation against real-world behavior

### Module 3 Learning Objectives
By the end of Module 3, students should be able to:
- Install and configure NVIDIA Isaac Sim
- Use Isaac ROS packages for perception and navigation
- Implement VSLAM algorithms
- Integrate navigation with perception systems

### Module 4 Learning Objectives
By the end of Module 4, students should be able to:
- Integrate Whisper for voice processing
- Implement LLM-based planning systems
- Create Vision-Language-Action pipelines
- Integrate all modalities into a cohesive system

## Assessment Strategies

### Formative Assessments (Weekly)
- Lab completion and functionality tests
- Code reviews and peer evaluations
- Concept understanding quizzes
- Implementation presentations

### Summative Assessments (Module-based)
- Module-specific projects demonstrating concepts
- Integration challenges across subsystems
- Technical documentation and reports
- Peer collaboration and contribution

### Capstone Assessment
- Complete Voice → Plan → Navigate → Perceive → Manipulate implementation
- Technical presentation of approach and results
- Written reflection on challenges and solutions
- Peer evaluation and feedback

### Grading Rubric Suggestions

#### Technical Implementation (40%)
- Code quality and documentation
- Correctness of implementation
- Efficient use of resources
- Innovation and problem-solving

#### Concept Understanding (30%)
- Application of theoretical concepts
- Problem-solving approach
- Integration of multiple components
- Troubleshooting and debugging skills

#### Collaboration and Communication (20%)
- Participation in discussions
- Peer collaboration effectiveness
- Presentation quality
- Technical documentation

#### Professional Skills (10%)
- Time management and project planning
- Following coding standards
- Proper use of version control
- Adherence to deadlines

## Laboratory Activities

### Lab Safety Guidelines
1. Ensure all electrical connections are secure before powering robots
2. Maintain a clear workspace to prevent accidents
3. Supervise robot operation, especially during testing
4. Have emergency stop procedures implemented
5. Proper lifting techniques when handling heavy equipment

### Week-by-Week Lab Activities
- **Week 1**: ROS 2 basics - publisher/subscriber implementation
- **Week 2**: Services and actions - request/response patterns
- **Week 3**: URDF and robot modeling - kinematic chains
- **Week 4**: Gazebo simulation environment setup
- **Week 5**: Sensor integration in simulation
- **Week 6**: Simulation-to-reality validation
- **Week 7**: Isaac Sim setup and basic scene
- **Week 8**: Perception pipeline with Isaac ROS
- **Week 9**: Navigation implementation with Nav2
- **Week 10**: Voice processing with Whisper
- **Week 11**: LLM-based planning integration
- **Week 12**: Complete VLA system integration
- **Week 13**: Capstone project demonstration

## Technical Setup for Instructors

### Classroom/Lab Configuration
1. **Workstation Requirements**: 
   - Minimum: 16GB RAM, quad-core CPU, RTX 3060
   - Recommended: 32GB+ RAM, multi-core CPU, RTX 4090
   - Network: Dedicated LAN for robot communication

2. **Robot Fleet Management**:
   - Charging stations with safety monitoring
   - Secure storage for robot transport
   - Spare batteries and accessories
   - Diagnostic tools for troubleshooting

3. **Software Management**:
   - Centralized image deployment for consistent environment
   - Version control for course materials
   - Automated testing environment
   - Backup and recovery procedures

### Troubleshooting Common Issues

#### ROS 2 Issues
- **Node Discovery**: Ensure ROS_DOMAIN_ID is consistent across systems
- **Clock Synchronization**: Use NTP for multi-device coordination
- **Resource Limits**: Increase system limits (ulimit) for intensive tasks

#### Simulation Issues
- **Performance**: Adjust rendering quality and physics accuracy
- **Model Loading**: Verify URDF and mesh file paths
- **Sensor Noise**: Configure appropriate noise models

#### Isaac Sim Issues
- **GPU Memory**: Monitor and limit simultaneous users
- **Scene Complexity**: Gradually increase scene complexity
- **Network Bandwidth**: Optimize for multi-user environment

## Inclusive Teaching Strategies

### Accommodation for Different Learning Styles
- **Visual Learners**: Use diagrams, 3D models, and video demonstrations
- **Kinesthetic Learners**: Provide hands-on robot interaction
- **Auditory Learners**: Encourage verbal explanations and discussions
- **Reading/Writing Learners**: Emphasize documentation and written reports

### Supporting Diverse Backgrounds
- Provide programming bootcamp for students with limited experience
- Offer additional math review for linear algebra concepts
- Create mixed-skill teams to promote peer learning
- Ensure all materials are available in accessible formats

## Course Adaptation Options

### Shortened Course (8 weeks)
- Combine Module 1 & 2 (4 weeks)
- Focus on core ROS 2 concepts
- Simulation with basic perception
- Simplified capstone project

### Extended Course (16 weeks)
- Add research project component
- Include advanced topics (reinforcement learning)
- Additional hardware platforms
- Industry guest lectures

### Online Delivery Options
- Use cloud-based GPU resources
- Provide recorded lab sessions
- Implement remote robot access
- Use simulation extensively

## Evaluation and Improvement

### Mid-Course Feedback
- Anonymous surveys on pace and content
- Focus groups on specific challenges
- Iterative improvements to materials
- Student suggestion implementation

### End-of-Course Evaluation
- Comprehensive feedback survey
- Learning objective achievement assessment
- Technical skill demonstration evaluation
- Course content effectiveness analysis

## Additional Resources

### Instructor Community
- Join Physical AI Educator Community
- Share experiences and materials
- Collaborate on curriculum updates
- Access to new resources and tools

### Continuing Education
- Attend robotics education conferences
- Participate in faculty development programs
- Engage with industry partners
- Pursue research collaborations

---

## Appendices

### Appendix A: Common Commands Reference
For quick reference during labs and demonstrations.

### Appendix B: Troubleshooting Flowchart
Decision tree for diagnosing common technical issues.

### Appendix C: Assessment Rubrics
Detailed rubrics for each assignment and milestone.

### Appendix D: Hardware Specifications
Detailed specifications for course-required hardware.

### Appendix E: Software Licenses
Information about required software licenses and availability.