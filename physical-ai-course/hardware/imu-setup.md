# IMU Sensor Integration Guide

## Overview
This guide provides instructions for integrating Inertial Measurement Unit (IMU) sensors with the Physical AI & Humanoid Robotics course. IMUs are critical for robot state estimation, navigation, and stability control.

## IMU Fundamentals

### What is an IMU?
An Inertial Measurement Unit (IMU) is an electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of accelerometers, gyroscopes, and magnetometers.

### IMU Components
- **Accelerometer**: Measures linear acceleration (3 axes: X, Y, Z)
- **Gyroscope**: Measures angular velocity (3 axes: roll, pitch, yaw)
- **Magnetometer**: Measures magnetic field (often used for compass heading)

### Common Applications
- Robot pose estimation and orientation
- Motion detection and gesture recognition
- Navigation and SLAM systems
- Stability control and balance

## Popular IMU Sensors

### Recommended Models
1. **SparkFun IMU Breakout - ICM-20948**
   - 9-axis: Accelerometer, Gyroscope, Magnetometer
   - I2C/SPI connectivity
   - ±2/4/8/16g Accelerometer
   - ±250/500/1000/2000 dps Gyroscope

2. **Adafruit LSM6DSOX + LIS3MDL**
   - 9-axis: Accelerometer + Gyroscope + Magnetometer
   - I2C interface
   - ±2/4/8/16g Accelerometer
   - ±125/250/500/1000/2000 dps Gyroscope
   - ±4/8/12/16 gauss Magnetometer

3. **Bosch BMX055**
   - 9-axis: Accelerometer, Gyroscope, Magnetometer
   - I2C/SPI interface
   - Ultra-low power consumption

4. **Vectornav VN-100**
   - Higher-end option with integrated AHRS
   - UART/USB/RS485 connectivity
   - Built-in orientation estimation

## Hardware Connection

### I2C Connection
Most IMU sensors use I2C for communication:

```
IMU Pin    →    Jetson/PC Pin
VCC        →    3.3V or 5V (check sensor voltage)
GND        →    GND
SDA        →    Pin 2 (GPIO 2) on Pi / I2C SDA pin
SCL        →    Pin 3 (GPIO 3) on Pi / I2C SCL pin
```

### SPI Connection
For higher-speed communication:

```
IMU Pin    →    Jetson/PC Pin
VCC        →    3.3V
GND        →    GND
MOSI       →    SPI MOSI
MISO       →    SPI MISO
SCK        →    SPI CLK
CS         →    GPIO pins (chip select)
```

### Pin Configuration for Raspberry Pi
```
VCC → Pin 1 (3.3V)
GND → Pin 6 (Ground)
SDA → Pin 3 (GPIO 2)
SCL → Pin 5 (GPIO 3)
```

## Software Installation

### 1. Enable I2C Interface
On Linux systems:
```bash
sudo raspi-config  # For Raspberry Pi
# Navigate: Interfacing Options → I2C → Enable
```

Or manually:
```bash
# Add to /boot/config.txt
echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
sudo modprobe i2c-dev
```

### 2. Check IMU Device Connection
```bash
# Install i2c tools
sudo apt update
sudo apt install -y i2c-tools

# Scan for connected I2C devices
sudo i2cdetect -y 1  # Use 0 for older Pi models
```

### 3. Install IMU Libraries
```bash
# Install Python libraries
pip3 install smbus2 adafruit-circuitpython-lsm6ds adafruit-circuitpython-lis3mdl

# Install ROS 2 packages
sudo apt install ros-humble-ros2-bridges ros-humble-imu-tools
```

### 4. Install RTIMULib2 (Comprehensive IMU Library)
```bash
git clone https://github.com/RPi-Distro/RTIMULib2.git
cd RTIMULib2/Linux
make -j$(nproc)
sudo make install
sudo ldconfig
```

## ROS 2 Integration

### 1. Create IMU ROS 2 Package
1. Create a new package for IMU handling:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python imu_sensor_integration
```

### 2. IMU Publisher Node
1. Create an IMU publisher node (`imu_sensor_integration/imu_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import time
import math

# For ICM-20948 IMU
try:
    import board
    import busio
    import adafruit_icm20x
except ImportError:
    print("Please install Adafruit libraries: pip3 install adafruit-circuitpython-icm20x")


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Create publisher
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # Initialize IMU
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.icm = adafruit_icm20x.ICM20948(i2c)
            self.get_logger().info('ICM20948 IMU initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize IMU: {e}')
            self.icm = None
        
        # Timer for publishing data
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100Hz
        
        self.get_logger().info('IMU Publisher node initialized')

    def publish_imu_data(self):
        if not self.icm:
            return
            
        try:
            # Read raw sensor data
            acceleration = self.icm.acceleration
            gyro = self.icm.gyro
            magnetic = self.icm.magnetic
            
            # Create and populate IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Set orientation (we'll estimate this later, or use a filter)
            # For now, we'll use zeros since we'll use orientation-less IMU message
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            
            # Set orientation covariance (unknown, so set to large value)
            imu_msg.orientation_covariance = [-1.0] + [0.0]*8  # Orientation unknown
            
            # Set angular velocity (rad/s)
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            
            # Set angular velocity covariance
            imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            
            # Set linear acceleration (m/s^2)
            imu_msg.linear_acceleration.x = acceleration[0]
            imu_msg.linear_acceleration.y = acceleration[1]
            imu_msg.linear_acceleration.z = acceleration[2]
            
            # Set linear acceleration covariance
            imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            
            # Publish IMU message
            self.imu_publisher.publish(imu_msg)
            
            # Create and populate magnetic field message
            mag_msg = MagneticField()
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.header.frame_id = 'imu_link'
            
            # Set magnetic field values (Tesla)
            mag_msg.magnetic_field.x = magnetic[0] / 1000000.0  # Convert nT to T
            mag_msg.magnetic_field.y = magnetic[1] / 1000000.0
            mag_msg.magnetic_field.z = magnetic[2] / 1000000.0
            
            # Set magnetic field covariance
            mag_msg.magnetic_field_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            
            # Publish magnetic field message
            self.mag_publisher.publish(mag_msg)
            
            self.get_logger().debug(f'Published IMU data: acc=({acceleration[0]:.3f}, {acceleration[1]:.3f}, {acceleration[2]:.3f})')
            
        except Exception as e:
            self.get_logger().error(f'Error reading IMU: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. IMU Orientation Estimation Node
1. Create a node to estimate orientation using filtering (`imu_sensor_integration/orientation_estimator.py`):

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
import quaternion  # pip3 install numpy-quaternion

class OrientationEstimator(Node):
    def __init__(self):
        super().__init__('orientation_estimator')
        
        # Subscribe to raw IMU data
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data_raw', self.imu_callback, 10)
        
        # Publish filtered IMU data
        self.filtered_publisher = self.create_publisher(Imu, 'imu/data_filtered', 10)
        
        # Initialize orientation (as quaternion)
        self.orientation = np.quaternion(1, 0, 0, 0)  # Identity rotation
        self.last_time = self.get_clock().now()
        
        # Filter parameters
        self.complementary_filter_alpha = 0.98
        self.magnetic_declination = 0.0  # Adjust for your location
        
        self.get_logger().info('Orientation Estimator initialized')

    def imu_callback(self, msg):
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        
        if dt <= 0:
            dt = 0.01  # Default to 100Hz if timing info is not available
        elif dt > 1.0:
            dt = 0.01  # Ignore large gaps in timing
            
        self.last_time = current_time
        
        # Extract angular velocity from IMU message
        gyro_vector = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        # Integrate angular velocity to estimate orientation change
        # Using small angle approximation: dq ≈ 0.5 * w * dt * q
        omega_quat = np.quaternion(0, *gyro_vector) * self.orientation
        delta_orientation = 0.5 * omega_quat * dt
        
        # Update orientation (simple integration)
        self.orientation = self.orientation + delta_orientation
        # Normalize the quaternion
        self.orientation = self.orientation.normalized()
        
        # Create output message by copying input but updating orientation
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.linear_acceleration = msg.linear_acceleration
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # Set the estimated orientation from integration
        filtered_msg.orientation.x = self.orientation.x
        filtered_msg.orientation.y = self.orientation.y
        filtered_msg.orientation.z = self.orientation.z
        filtered_msg.orientation.w = self.orientation.w
        
        # Set orientation covariance (estimate based on our calculation confidence)
        variance = 0.01 + dt * 0.01  # Increase uncertainty over time
        filtered_msg.orientation_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
        
        # Publish the filtered message
        self.filtered_publisher.publish(filtered_msg)
        
        self.get_logger().debug(f'Est. Orientation: ({self.orientation.x:.3f}, {self.orientation.y:.3f}, {self.orientation.z:.3f}, {self.orientation.w:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = OrientationEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Update setup.py
1. Update the package's setup.py to include entry points:

```python
from setuptools import find_packages, setup

package_name = 'imu_sensor_integration'

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
    description='Package for IMU sensor integration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_sensor_integration.imu_publisher:main',
            'orientation_estimator = imu_sensor_integration.orientation_estimator:main',
        ],
    },
)
```

### 5. Build and Test
1. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select imu_sensor_integration
source install/setup.bash
```

2. Run the IMU publisher:
```bash
ros2 run imu_sensor_integration imu_publisher
```

3. In another terminal, run the orientation estimator:
```bash
ros2 run imu_sensor_integration orientation_estimator
```

## Calibration

### 1. Accelerometer Calibration
1. Place your robot/imu in six different positions:
   - +X up, -X up
   - +Y up, -Y up
   - +Z up, -Z up
2. Record the raw values and calculate offsets

### 2. Gyroscope Calibration
1. Keep the IMU stationary
2. Take multiple readings and average to find bias
3. Subtract bias from all future readings

### 3. Magnetometer Calibration
1. Rotate the IMU in all directions (figure-8 motion)
2. Record min/max values on each axis
3. Calculate offsets for hard iron correction

## Filtering Techniques

### 1. Complementary Filter
A complementary filter combines gyroscope (high-frequency) and accelerometer/magnetometer (low-frequency) data:

```python
# Simplified complementary filter
alpha = 0.98  # Filter constant
rate = dt * gyro_reading
acc_angle = atan2(acc_y, acc_z)  # From accelerometer
filtered_angle = alpha * (filtered_angle + rate) + (1 - alpha) * acc_angle
```

### 2. Kalman Filter
For more sophisticated filtering, implement an Extended Kalman Filter (EKF) to fuse IMU data optimally.

### 3. Madgwick or Mahony Filter
These are specialized filters for IMU data fusion:
- Madgwick: Lower computational load, good for most applications
- Mahony: Also computationally efficient, good alternative

## Troubleshooting

### Common Issues
1. **Drifting readings**: Calibrate sensors, check for vibrations
2. **Noisy data**: Apply low-pass filters, check connections
3. **Incorrect orientation**: Verify coordinate system definitions
4. **Temperature effects**: Some IMUs have temperature compensation

### Checking Data Quality
1. Plot raw IMU data over time
2. Look for sudden jumps or unrealistic values
3. Monitor statistical measures (mean, variance)
4. Compare with other sensors when available

## Integration with Robotics Stack

### 1. Robot Localization
- Use IMU data with other sensors in robot_localization package
- Combine with wheel encoders and visual odometry

### 2. Navigation
- Provide orientation data to navigation stack
- Use for obstacle avoidance and path planning

### 3. Control Systems
- Use for feedback control of robot balance
- Implement attitude controllers for aerial vehicles

## Performance Optimization

### 1. Sampling Rate
- Choose appropriate sampling rate (typically 50-200Hz)
- Higher rates for dynamic applications
- Lower rates for battery-powered systems

### 2. Power Management
- Many IMUs have power saving modes
- Use sleep modes when robot is idle
- Optimize polling vs interrupt-based reading

## Security and Safety

### Data Integrity
- Implement checksums for critical applications
- Monitor for sensor failures
- Implement fault tolerance in control systems

### Physical Protection
- Secure IMU mounting to prevent vibration
- Protect from electromagnetic interference
- Shield from radio frequency interference

## Maintenance

### Regular Checks
1. Verify sensor calibration periodically
2. Check physical mounting and wiring
3. Monitor data quality metrics
4. Update IMU firmware if available