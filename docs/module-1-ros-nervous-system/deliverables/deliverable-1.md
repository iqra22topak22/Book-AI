# Module 1 Deliverables: ROS 2 Nervous System

## Overview
This document outlines the deliverables for Module 1 of the Physical AI & Humanoid Robotics course. These deliverables demonstrate your understanding of ROS 2 fundamentals, node creation, and inter-node communication.

## Deliverable 1.1: Basic ROS 2 Package
**Due Date**: End of Week 1  
**Weight**: 10% of Module 1 grade

### Requirements
Create a ROS 2 package that demonstrates fundamental ROS 2 concepts:
1. Contains at least 3 different types of nodes:
   - Publisher node
   - Subscriber node
   - Service server/client
2. Implements custom message types
3. Uses parameters for configuration
4. Includes proper documentation and README
5. Builds successfully using colcon

### Submission Requirements
- Complete package source code
- README.md with build and run instructions
- Screenshot of successful execution
- Brief written explanation of design choices

### Evaluation Criteria
- Code functionality (40%)
- Proper ROS 2 architecture implementation (30%)
- Documentation quality (20%)
- Code organization and style (10%)

## Deliverable 1.2: Robot Publisher-Subscriber System
**Due Date**: End of Week 2  
**Weight**: 15% of Module 1 grade

### Requirements
Create a system that simulates controlling a simple robot with publish-subscribe communication:
1. Design at least 3 custom message types for robot communication
2. Implement nodes for:
   - Robot state publisher
   - Command subscriber
   - Sensor data publisher
3. Include proper error handling and validation
4. Demonstrate the system with simulated robot state

### Submission Requirements
- Complete source code
- Architecture diagram
- Test results showing successful communication
- Performance analysis of message passing

### Evaluation Criteria
- System design quality (30%)
- Implementation correctness (30%)
- Message design and efficiency (20%)
- Error handling and robustness (20%)

## Deliverable 1.3: URDF Robot Model
**Due Date**: End of Week 3  
**Weight**: 20% of Module 1 grade

### Requirements
Create a complete URDF model for a simple robot and integrate it with ROS 2:
1. Create a detailed URDF file for a robot with at least 5 joints
2. Include visual and collision models
3. Add proper material definitions
4. Integrate with ROS 2 for state publishing
5. Include a launch file to visualize the robot in RViz

### Submission Requirements
- URDF files
- ROS 2 integration code
- Launch files
- Visualization screenshots
- Technical report on model decisions

### Evaluation Criteria
- URDF model completeness (35%)
- Integration with ROS 2 (30%)
- Visualization quality (20%)
- Technical report quality (15%)

## General Submission Guidelines
- All code must be properly documented
- Submit as a zipped package with clear directory structure
- Include a README with build and run instructions
- Code must be compatible with ROS 2 Humble Hawksbill
- Use Python 3.10+ or C++17 as appropriate
- Follow ROS 2 style guidelines (PEP 8 for Python, Google style for C++)

## Grading Rubric
- Exceeds Expectations (A): All requirements met with advanced implementation
- Meets Expectations (B): All requirements met with effective implementation
- Approaches Expectations (C): Most requirements met with basic implementation
- Below Expectations (D/F): Requirements not adequately met

## Late Submission Policy
- Within 24 hours: 10% grade reduction
- Within 48 hours: 25% grade reduction
- Beyond 48 hours: Not accepted without prior approval

## Academic Integrity
All deliverables must be your own work. You may use online resources and documentation but must cite them appropriately. Code sharing with other students is not permitted.