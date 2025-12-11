# Intel RealSense Camera Integration Guide

## Overview
This guide provides instructions for integrating Intel RealSense cameras with the Physical AI & Humanoid Robotics course. RealSense cameras provide RGB-D (color + depth) perception capabilities essential for robotics applications.

## Supported Cameras

### Recommended Models
- **Intel RealSense D455**: Latest model with enhanced features
- **Intel RealSense D435i**: Includes IMU sensors
- **Intel RealSense D435**: Baseline depth camera model

### Specifications Comparison
| Model | Resolution | FPS | Field of View | IMU | Connectivity |
|-------|------------|-----|---------------|-----|--------------|
| D455 | 1280×720 | 90fps | 86°×57° | Yes | USB 3.2 Gen 1 |
| D435i | 1280×720 | 30fps | 86°×54° | Yes | USB 3.2 Gen 1 |
| D435 | 1280×720 | 30fps | 85°×58° | No | USB 3.2 Gen 1 |

## Hardware Setup

### 1. Physical Connection
1. Connect the RealSense camera to your workstation via USB 3.2 Gen 1 port (blue connector)
2. Ensure the USB port is powered and functional
3. Verify camera LED indicators (power and depth illumination)
4. Position the camera at appropriate height and angle for your application

### 2. Mounting Options
- Tripod mount (1/4"-20 thread)
- Custom mounting brackets
- Robot mounting solutions

## Software Installation

### 1. Ubuntu Dependencies
1. Install required packages:
```bash
sudo apt update
sudo apt install -y libssl-dev libusb-1.0-0-dev pkg-config
sudo apt install -y libgtk-3-dev
sudo apt install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

### 2. Librealsense Library
1. Clone the repository:
```bash
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
```

2. Install dependencies and build:
```bash
sudo scripts/setup_udev_rules.sh
sudo scripts/install_backend_udev_rules.sh

# Create build directory
mkdir build && cd build

# Configure build
cmake ../ \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DFORCE_RSUSB_BACKEND=false \
    -DBUILD_PYTHON_BINDINGS=bool:true

# Compile
make -j$(nproc)
```

3. Install the library:
```bash
sudo make install
```

### 3. Verification
1. Run the viewer to test the camera:
```bash
# Connect camera first, then run:
realsense-viewer
```

2. Check if camera is detected:
```bash
rs-enumerate-devices
```

## ROS 2 Integration

### 1. Install RealSense ROS 2 Package
1. Create workspace if you don't have one:
```bash
mkdir -p ~/ros2_realsense_ws/src
cd ~/ros2_realsense_ws/src
```

2. Clone the RealSense ROS 2 package:
```bash
git clone -b humble https://github.com/IntelRealSense/realsense-ros.git
```

3. Build the package:
```bash
cd ~/ros2_realsense_ws
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source install/setup.bash
```

### 2. Launch RealSense Node
1. Test the launch file:
```bash
source ~/ros2_realsense_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

2. View the camera data:
```bash
# View RGB image
ros2 run rqt_image_view rqt_image_view

# View depth image
# In rqt_image_view, select /camera/depth/image_rect_raw

# List all topics
ros2 topic list
```

### 3. Camera Parameters Configuration
1. Create a custom configuration file `d435_config.yaml`:
```yaml
camera:
  ros__parameters:
    serial_no: ""
    usb_port_id: ""
    device_type: 'd435'
    enable_depth: true
    enable_color: true
    enable_infra1: false
    enable_infra2: false
    enable_gyro: true
    enable_accel: true
    unite_imu_method: "linear_interpolation"
    depth_module.profile: '1280x720x30'
    rgb_camera.profile: '1280x720x30'
    enable_pointcloud: true
    pointcloud_texture_stream: RS2_STREAM_COLOR
    filters: 'pointcloud'
    calib_odom_file: ""
    publish_odom_tf: true
    publish_tf: true
    qos_image: 2
    qos_info: 2
```

2. Launch with custom configuration:
```bash
ros2 launch realsense2_camera rs_launch.py config_file:="path/to/d435_config.yaml"
```

## Python Integration

### 1. Install Python Bindings
1. Install the Python wrapper:
```bash
cd ~/workspace/librealsense/build
sudo make install-python3
```

2. Install through pip:
```bash
pip3 install pyrealsense2
```

### 2. Basic Python Usage
1. Test basic functionality:
```python
import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(1) & 0xFF
        
        # Press esc or 'q' to quit
        if key == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
            
finally:
    # Stop streaming
    pipeline.stop()
```

### 3. ROS 2 Node with RealSense
1. Create a ROS 2 node that processes RealSense data:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        
        # Create publishers for depth and color images
        self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)
        self.color_publisher = self.create_publisher(Image, 'color_image', 10)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Configure and start RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.pipeline.start(config)
        self.timer = self.create_timer(0.1, self.publish_images)  # 10Hz
        
        self.get_logger().info('RealSense Node initialized')

    def publish_images(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                self.get_logger().warning('No frames received')
                return

            # Convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Convert to ROS Image messages
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='mono16')
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

            # Add timestamps and frame IDs
            timestamp = self.get_clock().now().to_msg()
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = 'camera_color_optical_frame'

            # Publish images
            self.depth_publisher.publish(depth_msg)
            self.color_publisher.publish(color_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing images: {e}')

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    
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

## Calibration

### 1. Intrinsic Calibration
1. Use the RealSense viewer for basic calibration check:
```bash
realsense-viewer
```

2. For more precise calibration, use Kalibr or other ROS packages:
```bash
# Install Kalibr
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/kalibr.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 2. Extrinsic Calibration with Other Sensors
1. For multi-camera setups, calibrate relative positions:
```bash
# Use ROS calibration tools
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/color/image_raw
```

## Performance Optimization

### 1. Resource Management
1. Monitor camera performance:
```bash
# Check camera bandwidth usage
sudo lsusb -v | grep -A 20 -i realsense
```

2. Adjust quality settings for performance requirements:
```bash
# Reduce resolution for higher FPS
rs-config --enable-stream depth --width 640 --height 480 --fps 60
```

### 2. Depth Accuracy
1. Fine-tune depth presets for your use case:
```bash
# Available presets: Default, Hand, HighAccuracy, HighDensity, MiddleRange, RemoveIR
rs-config --visual-preset HighAccuracy
```

## Troubleshooting

### Common Issues
1. **Camera not detected**: Check USB connection and power. Try different USB port.
2. **Permission denied**: Run `sudo chmod a+rw /dev/bus/usb/*/*/` or add udev rules.
3. **High CPU usage**: Reduce resolution or frame rate.
4. **Depth noise**: Ensure adequate lighting and appropriate preset.

### Udev Rules (Linux)
1. If experiencing permission issues, ensure udev rules are properly installed:
```bash
# This should have been done during installation
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Integration Scenarios

### 1. Object Detection
- Use depth information for 3D object localization
- Combine RGB and depth for enhanced object detection
- Implement collision avoidance based on depth maps

### 2. Navigation and Mapping
- Generate 3D point clouds for mapping
- Use depth for obstacle detection in navigation
- Implement SLAM with RGB-D data

### 3. Manipulation
- Estimate object poses for grasping
- Verify manipulation success with depth feedback
- Create 3D models of objects for planning

## Advanced Topics

### 1. Multi-Camera Setup
1. Use multiple RealSense cameras for panoramic vision:
```bash
# Launch multiple cameras with different serial numbers
ros2 launch realsense2_camera rs_multiple_devices.launch.py
```

### 2. Post-Processing
1. Apply filters for improved depth quality:
```python
# Depth post-processing filters
decimation = rs.decimation_filter()
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()
hole_filling = rs.hole_filling_filter()

# Apply filters in pipeline
filtered_depth = hole_filling.process(temporal.process(spatial.process(decimation.process(depth_frame))))
```

## Security Considerations

### Data Privacy
- Be mindful of privacy when using cameras
- Consider blurring or masking sensitive areas
- Secure camera feeds if transmitting over networks

### Physical Security
- Protect camera hardware from damage
- Secure mounting to prevent tampering
- Implement authentication for camera access

## Maintenance

### Regular Checks
1. Clean camera lenses regularly with appropriate materials
2. Update firmware periodically:
```bash
rs-fw-update -l  # List available firmware updates
rs-fw-update -f <firmware_file>  # Update firmware
```

2. Test depth accuracy with known objects periodically
3. Verify network performance if streaming over network