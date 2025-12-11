# Cloud Infrastructure Templates for Ether Lab

## Overview
This document provides templates for cloud infrastructure using AWS g5/g6e instances and Omniverse for the Physical AI & Humanoid Robotics course. It includes setup guides, configuration files, and connection protocols.

## AWS Infrastructure Template

### EC2 Instance Configuration (g5/g6e)

```yaml
# AWS CloudFormation Template
AWSTemplateFormatVersion: '2010-09-09'
Description: 'Infrastructure for Physical AI & Humanoid Robotics Course'

Parameters:
  InstanceType:
    Type: String
    Default: g5.2xlarge
    AllowedValues:
      - g5.xlarge
      - g5.2xlarge
      - g5.4xlarge
      - g5.8xlarge
      - g6e.xlarge
      - g6e.2xlarge
      - g6e.4xlarge
      - g6e.8xlarge
    Description: EC2 GPU instance type for simulation workloads

Resources:
  EtherLabInstance:
    Type: AWS::EC2::Instance
    Properties:
      ImageId: ami-0abcdef1234567890  # Ubuntu 22.04 LTS with NVIDIA drivers
      InstanceType: !Ref InstanceType
      SecurityGroupIds:
        - !Ref EtherLabSecurityGroup
      KeyName: !Ref KeyPair
      Tags:
        - Key: Name
          Value: PhysicalAI-EtherLab

  EtherLabSecurityGroup:
    Type: AWS::EC2::SecurityGroup
    Properties:
      GroupDescription: Security group for Ether Lab instances
      SecurityGroupIngress:
        - IpProtocol: tcp
          FromPort: 22
          ToPort: 22
          CidrIp: !Ref AllowedCIDR
        - IpProtocol: tcp
          FromPort: 80
          ToPort: 80
          CidrIp: !Ref AllowedCIDR
        - IpProtocol: tcp
          FromPort: 443
          ToPort: 443
          CidrIp: !Ref AllowedCIDR
        - IpProtocol: tcp
          FromPort: 8080
          ToPort: 8080
          CidrIp: !Ref AllowedCIDR

Outputs:
  InstanceId:
    Description: ID of the Ether Lab Instance
    Value: !Ref EtherLabInstance
  PublicIP:
    Description: Public IP address of the Ether Lab Instance
    Value: !GetAtt EtherLabInstance.PublicIp
```

### Instance Setup Script

```bash
#!/bin/bash
# Ether Lab Setup Script

# Update system and install dependencies
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git curl wget

# Install NVIDIA drivers (for g5/g6e instances)
sudo apt install -y nvidia-driver-535 nvidia-utils-535

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install Docker Compose
sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Install ROS 2 Humble Hawksbill
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US.UTF-8
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-nav2-bringup

# Install Python dependencies
pip3 install numpy scipy torch openai-whisper

# Setup ROS environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Reboot to load NVIDIA drivers
sudo reboot
```

## Omniverse Setup Template

### Omniverse Asset Configuration

```
omniverse/
├── assets/
│   ├── robots/
│   │   ├── unitree/
│   │   └── hiwonder/
│   ├── environments/
│   │   ├── indoor/
│   │   └── outdoor/
│   └── sensors/
│       ├── realsense/
│       └── lidar/
└── scenes/
    ├── module-1-scene.usd
    ├── module-2-scene.usd
    ├── module-3-scene.usd
    └── module-4-scene.usd
```

### Docker Compose for Omniverse

```yaml
version: '3.8'

services:
  omniverse-client:
    image: nvidia/ov/raycast:2023.1.1
    container_name: omniverse-client
    privileged: true
    environment:
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./omniverse:/omniverse:rw
    devices:
      - /dev/dri:/dev/dri
    ports:
      - "8211:8211"
    command: ["--exec", "omnishm"]

  ros-bridge:
    image: nvidia/isaac-sim:latest
    container_name: ros-bridge
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - ./omniverse:/omniverse:rw
    ports:
      - "9090:9090"
    command: ["--/headless=true", "--/renderer=RayTracedLighting", "--active-script=/scripts/ros_bridge.py"]
```

## Connection Protocols

### Local to Cloud Communication

```python
# Connection protocol example
import socket
import json

class CloudConnection:
    def __init__(self, cloud_ip, cloud_port):
        self.cloud_ip = cloud_ip
        self.cloud_port = cloud_port
        self.socket = None

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.cloud_ip, self.cloud_port))
        return True

    def send_command(self, command):
        if self.socket:
            self.socket.send(json.dumps(command).encode())
            response = self.socket.recv(4096)
            return json.loads(response.decode())
        return None

    def close(self):
        if self.socket:
            self.socket.close()
```

## Terraform Template

```hcl
# terraform/main.tf
terraform {
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }
}

provider "aws" {
  region = var.aws_region
}

resource "aws_instance" "ether_lab" {
  ami           = var.ami_id
  instance_type = var.instance_type

  root_block_device {
    volume_size = 100
    volume_type = "gp3"
  }

  vpc_security_group_ids = [aws_security_group.ether_lab_sg.id]
  key_name               = var.key_name

  tags = {
    Name = "PhysicalAI-EtherLab"
  }

  user_data = base64encode(templatefile("setup.sh", {
    ros_version = var.ros_version
  }))
}

resource "aws_security_group" "ether_lab_sg" {
  name_prefix = "ether-lab-"

  ingress {
    from_port   = 22
    to_port     = 22
    protocol    = "tcp"
    cidr_blocks = var.allowed_cidr_blocks
  }

  ingress {
    from_port   = 8080
    to_port     = 8080
    protocol    = "tcp"
    cidr_blocks = var.allowed_cidr_blocks
  }

  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }
}

output "instance_public_ip" {
  value = aws_instance.ether_lab.public_ip
}
```

```hcl
# terraform/variables.tf
variable "aws_region" {
  description = "AWS region for deployment"
  type        = string
  default     = "us-west-2"
}

variable "instance_type" {
  description = "EC2 instance type"
  type        = string
  default     = "g5.2xlarge"
}

variable "ami_id" {
  description = "AMI ID for Ubuntu 22.04 LTS with NVIDIA drivers"
  type        = string
}

variable "key_name" {
  description = "EC2 Key Pair name"
  type        = string
}

variable "allowed_cidr_blocks" {
  description = "CIDR blocks allowed to access the instance"
  type        = list(string)
  default     = ["0.0.0.0/0"]
}

variable "ros_version" {
  description = "ROS version to install"
  type        = string
  default     = "humble"
}
```

## Security Considerations

- Use IAM roles instead of access keys where possible
- Implement VPC with private subnets for sensitive operations
- Enable AWS CloudTrail for API monitoring
- Use AWS Systems Manager for secure instance management
- Encrypt EBS volumes and S3 storage

## Cost Management

- Use Spot Instances for development and testing
- Implement auto-scaling based on demand
- Set up billing alerts and budgets
- Terminate unused instances automatically