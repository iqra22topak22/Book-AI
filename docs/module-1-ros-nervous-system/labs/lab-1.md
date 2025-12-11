# Lab 1.1: Creating Your First ROS 2 Package and Nodes

## Overview
This lab provides hands-on experience creating a ROS 2 package and implementing basic publisher and subscriber nodes. You'll create a simple "Hello World" style example with nodes that publish and subscribe to messages.

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Basic understanding of ROS 2 concepts from Lesson 1.1

## Learning Objectives
By the end of this lab, students will be able to:
- Create a ROS 2 package using colcon
- Implement a publisher node in Python
- Implement a subscriber node in Python
- Use rclpy to create ROS 2 nodes
- Build and run ROS 2 nodes

## Lab Duration
Estimated time: 2-3 hours

## Step-by-Step Instructions

### Step 1: Create a Workspace and Package
1. Create a new workspace directory:
```bash
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws
```

2. Create a new ROS 2 package named `physical_ai_examples`:
```bash
cd ~/physical_ai_ws/src
ros2 pkg create --build-type ament_python physical_ai_examples
```

3. Your package structure should look like:
```
physical_ai_examples/
├── package.xml
├── setup.cfg
├── setup.py
└── physical_ai_examples/
    └── __init__.py
```

### Step 2: Create Publisher Node
1. In the `physical_ai_examples/physical_ai_examples/` directory, create a new file called `publisher_member_function.py`:

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

### Step 3: Create Subscriber Node
1. In the same directory, create a new file called `subscriber_member_function.py`:

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

### Step 4: Update Package Setup
1. Modify the `setup.py` file in the root of your package to include the entry points for your nodes:

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

### Step 5: Build and Test
1. Go to your workspace root and build the package:
```bash
cd ~/physical_ai_ws
colcon build --packages-select physical_ai_examples
```

2. Source your workspace:
```bash
source install/setup.bash
```

3. Open a new terminal and run the publisher node:
```bash
source ~/physical_ai_ws/install/setup.bash
ros2 run physical_ai_examples talker
```

4. In another terminal, run the subscriber node:
```bash
source ~/physical_ai_ws/install/setup.bash
ros2 run physical_ai_examples listener
```

5. You should see the publisher sending messages and the subscriber receiving them.

### Step 6: Experiment and Extend
1. Modify the message content in the publisher to send different information
2. Change the timer period to publish at different intervals
3. Add a parameter to control the message content
4. Try using different message types (e.g., Int32, Float32)

## Deliverable
Submit a report containing:
1. Screenshots of your publisher and subscriber nodes running
2. Modified code that demonstrates at least one extension from Step 6
3. Explanation of what your modifications do
4. Any challenges you encountered and how you resolved them

## Assessment Criteria
- Code compiles and runs without errors (30%)
- Publisher and subscriber successfully communicate (30%)
- Code modifications are meaningful and well-documented (25%)
- Report is comprehensive and well-written (15%)

## Additional Resources
- [ROS 2 Python Node Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [rclpy Documentation](https://docs.ros2.org/en/humble/p/rclpy/)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html)