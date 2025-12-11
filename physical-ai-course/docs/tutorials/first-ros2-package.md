---
sidebar_position: 8
---

# Tutorial: Setting Up Your First ROS 2 Package

## Overview
This tutorial walks you through creating your first ROS 2 package for the Physical AI course. You'll learn to create a simple publisher-subscriber system that demonstrates fundamental ROS 2 concepts.

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Basic understanding of Linux command line
- Completed Module 1 lessons on ROS 2 fundamentals

## Learning Objectives
By the end of this tutorial, you will be able to:
- Create a new ROS 2 package
- Implement a publisher node
- Implement a subscriber node
- Build and run your nodes
- Understand the publisher-subscriber pattern

## Step 1: Create Your Workspace

1. Create a new ROS 2 workspace for this course:
```bash
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws
```

2. Your workspace structure should look like:
```
physical_ai_ws/
└── src/
```

## Step 2: Create the Package

1. Navigate to your source directory:
```bash
cd ~/physical_ai_ws/src
```

2. Create a new ROS 2 package called `physical_ai_examples`:
```bash
ros2 pkg create --build-type ament_python physical_ai_examples
```

3. Your package structure looks like this:
```
physical_ai_examples/
├── package.xml
├── setup.cfg
├── setup.py
└── physical_ai_examples/
    └── __init__.py
```

## Step 3: Implement the Publisher Node

1. Inside the `physical_ai_examples/physical_ai_examples` directory, create a file called `publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

2. This code creates a publisher node that:
   - Publishes a String message to the topic named `topic`
   - Publishes every 0.5 seconds
   - Increments a counter in the message content

## Step 4: Implement the Subscriber Node

1. Create `subscriber_member_function.py` in the same directory:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

2. This code creates a subscriber node that:
   - Subscribes to the `topic` topic
   - Listens for String messages
   - Logs the received message content

## Step 5: Update the Package Setup

1. Modify the `setup.py` file to include entry points for your nodes:

```python
from setuptools import find_packages, setup

package_name = 'physical_ai_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Examples for Physical AI course',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = physical_ai_examples.publisher_member_function:main',
            'listener = physical_ai_examples.subscriber_member_function:main',
        ],
    },
)
```

## Step 6: Build and Run the Package

1. Go to your workspace root and build the package:
```bash
cd ~/physical_ai_ws
colcon build --packages-select physical_ai_examples
```

2. Source your workspace environment:
```bash
source install/setup.bash
```

3. Open a new terminal and run the publisher:
```bash
source ~/physical_ai_ws/install/setup.bash
ros2 run physical_ai_examples talker
```

4. In another terminal, run the subscriber:
```bash
source ~/physical_ai_ws/install/setup.bash
ros2 run physical_ai_examples listener
```

5. You should see the publisher sending messages and the subscriber receiving them.

## Step 7: Experiment and Extend

1. Modify the timer period in the publisher to change the publishing rate
2. Add a parameter to the publisher to customize the message
3. Create a second subscriber that logs to a file instead of console

## Troubleshooting

If you encounter issues:

1. **Nodes don't communicate**: Verify both terminals have sourced the setup.bash file
2. **Build errors**: Check that your Python syntax is correct
3. **Node not found**: Ensure the package has been built successfully

```bash
# Check if the executable was created
find ~/physical_ai_ws/install -name "talker"
```

## Summary

In this tutorial, you learned to:
- Create a ROS 2 package from scratch
- Implement publisher and subscriber nodes
- Use the rclpy library for ROS 2 programming
- Build and run your nodes

This foundational knowledge will help you create more complex ROS 2 applications throughout the course. The publisher-subscriber pattern is a core concept that you'll use in various contexts.

## Next Steps

- Try creating a service server and client
- Learn to pass parameters to your nodes
- Explore ROS 2 message types beyond String