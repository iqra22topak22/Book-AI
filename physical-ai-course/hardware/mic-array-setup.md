# Microphone Array Setup Guide

## Overview
This guide provides instructions for setting up microphone arrays for the Physical AI & Humanoid Robotics course. Microphone arrays are essential for voice command processing, spatial audio processing, and noise reduction in robotic applications.

## Microphone Array Fundamentals

### What is a Microphone Array?
A microphone array is a set of microphones placed in a specific geometric configuration to achieve desired spatial selectivity. In robotics, these are used for:
- Voice command recognition
- Sound source localization
- Noise reduction and beamforming
- Speaker identification

### Types of Arrays
1. **Linear Arrays**: Microphones arranged in a line
2. **Circular Arrays**: Microphones arranged in a circle
3. **Planar Arrays**: Microphones arranged in a 2D grid
4. **Spherical Arrays**: 3D arrangement for full spatial coverage

### Array Benefits
- Improved signal-to-noise ratio
- Directional sensitivity (beamforming)
- Sound source localization
- Echo cancellation

## Recommended Hardware

### 1. Matrix Voice
- 8 microphones in far-field circular array
- Built-in FPGA for beamforming
- USB interface
- Circular form factor (10mm spacing)

### 2. ReSpeaker Mic Array
- 6 or 4 microphone options
- Far-field voice capture
- USB connectivity
- Circular design optimized for voice

### 3. PS3 Eye with MEMS Microphones
- Can be modified with external MEMS mics
- USB interface
- 4 microphone array option

### 4. USB Microphone Arrays
- Plug-and-play options
- Various configurations available
- Good for prototyping

## Hardware Setup

### 1. Matrix Voice Setup
#### Physical Connection
1. Connect the Matrix Voice to your robot's computer via USB cable
2. Ensure the microphones face outward for omnidirectional capture (if using circular array)
3. Position away from fans, motors, and other noise sources

#### Power Requirements
- Powered via USB (5V)
- Power consumption: ~500mA
- No external power needed

### 2. ReSpeaker Setup
#### Physical Connection
1. Connect via USB or SPI depending on model
2. Position array facing toward expected user direction
3. Secure mounting to prevent vibrations

#### Power Requirements
- Can be powered via USB or headers

### 3. USB Audio Interface Setup
#### Connection
1. Connect USB audio interface to your system
2. Verify it's recognized by the OS:
```bash
lsusb | grep -i audio
arecord -l  # List recording devices
```

## Software Installation

### 1. ALSA (Advanced Linux Sound Architecture)
```bash
sudo apt update
sudo apt install alsa-utils alsa-tools alsa-tools-gui pulseaudio
```

### 2. Audio Drivers
```bash
# For Matrix Creator/VOICE
git clone https://github.com/matrix-io/matrix-creator-alsa.git
cd matrix-creator-alsa
make
sudo make install
```

### 3. Audio Capture Libraries
```bash
pip3 install pyaudio sounddevice numpy scipy

# For advanced audio processing
pip3 install librosa webrtcvad
```

### 4. Check Audio Devices
```bash
# List capture devices
arecord -l

# Test a specific device
arecord -D hw:1,0 -f cd test.wav
# Ctrl+C to stop
```

## ROS 2 Integration

### 1. Create Audio Package
1. Create ROS 2 package for audio processing:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python audio_processing
```

### 2. Audio Publisher Node
1. Create an audio publisher node (`audio_processing/audio_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import pyaudio
import numpy as np
import threading
import queue


class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        
        # Create publisher for audio data
        self.audio_publisher = self.create_publisher(AudioData, 'audio_input', 10)
        
        # Audio parameters
        self.rate = 16000  # Sample rate for Whisper
        self.chunk = 1024  # Frames per buffer
        self.format = pyaudio.paInt16  # 16-bit samples
        self.channels = 4  # For 4-channel array (adjust as needed)
        
        # Setup PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Open audio stream
        try:
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
            self.get_logger().info(f"Opened audio stream: {self.channels} channels at {self.rate}Hz")
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}")
            raise
        
        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self.capture_audio, daemon=True)
        self.running = True
        self.capture_thread.start()
        
        self.get_logger().info("Audio Publisher initialized")

    def capture_audio(self):
        """Continuously capture audio and publish to ROS topic"""
        while self.running:
            try:
                # Read audio data from stream
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                
                # Create and publish audio message
                msg = AudioData()
                msg.data = data
                self.audio_publisher.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f"Error during audio capture: {e}")
                break

    def destroy_node(self):
        """Clean up audio resources upon node destruction"""
        self.running = False
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    
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

### 3. Beamforming Node
1. Create a beamforming node (`audio_processing/beamforming.py`):

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
import scipy.signal as signal


class BeamformingNode(Node):
    def __init__(self):
        super().__init__('beamforming_node')
        
        # Subscribe to raw audio data
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )
        
        # Publish processed audio and source localization
        self.processed_pub = self.create_publisher(AudioData, 'audio_processed', 10)
        self.source_pub = self.create_publisher(PointCloud, 'sound_source_location', 10)
        
        # Array configuration (adjust based on your hardware)
        # Positions in meters relative to center
        self.array_positions = np.array([
            [0.0, 0.0, 0.0],    # Channel 0 (center)
            [0.04, 0.0, 0.0],   # Channel 1
            [0.0, 0.04, 0.0],   # Channel 2
            [-0.04, 0.0, 0.0]   # Channel 3
        ])
        
        # Processing parameters
        self.sample_rate = 16000
        self.frame_size = 1024
        self.overlap = 0.5
        self.fft_size = 512
        self.speed_of_sound = 343  # m/s
        
        # Buffer for storing frames
        self.channel_buffers = [[] for _ in range(4)]  # Assuming 4 channels
        
        self.get_logger().info("Beamforming node initialized")

    def audio_callback(self, msg):
        """Process incoming multi-channel audio data"""
        try:
            # Convert bytes to numpy arrays (for 16-bit samples)
            audio_bytes = np.frombuffer(msg.data, dtype=np.int16)
            
            # Reshape based on number of channels (assuming 4 channels)
            if len(audio_bytes) % 4 != 0:
                self.get_logger().warn("Audio data not divisible by channel count")
                return
                
            samples_per_channel = len(audio_bytes) // 4
            reshaped = audio_bytes.reshape(samples_per_channel, 4)
            
            # Apply beamforming
            enhanced_signal = self.apply_beamforming(reshaped)
            
            # Publish enhanced audio
            enhanced_msg = AudioData()
            enhanced_msg.data = enhanced_signal.tobytes()
            self.processed_pub.publish(enhanced_msg)
            
            # Localize sound source
            source_direction = self.localize_sound_source(reshaped)
            
            # Publish localization
            if source_direction is not None:
                self.publish_source_location(source_direction)
                
        except Exception as e:
            self.get_logger().error(f"Error in audio callback: {e}")

    def apply_beamforming(self, multi_channel_audio):
        """
        Apply delay-and-sum beamforming to enhance signal from specific direction
        """
        num_samples, num_channels = multi_channel_audio.shape
        
        # For simplicity, using a simple delay-and-sum beamformer
        # In practice, you might want to use MVDR, MUSIC, or other advanced methods
        
        # For this example, we'll just sum the channels after compensating for delays
        # toward the front of the robot (0 degrees azimuth)
        
        # Calculate steering delays for frontal direction (0, 0, 1) for each mic
        look_direction = np.array([0, 0, 1])  # Look straight ahead
        delays = self.calculate_steering_delays(look_direction)
        
        # Apply delays
        enhanced_signal = np.zeros(num_samples)
        
        for ch_idx in range(num_channels):
            delay_samples = int(delays[ch_idx] * self.sample_rate)
            
            if delay_samples >= 0:
                # Delay the signal by shifting
                delayed_signal = np.zeros(num_samples)
                delayed_signal[delay_samples:] = multi_channel_audio[:-delay_samples if delay_samples > 0 else None, ch_idx]
            else:
                # Advance the signal
                delayed_signal = np.zeros(num_samples)
                delayed_signal[:delay_samples] = multi_channel_audio[-delay_samples:, ch_idx]
            
            enhanced_signal += delayed_signal / num_channels  # Average
        
        # Convert back to int16 format
        enhanced_signal = np.clip(enhanced_signal, -32768, 32767).astype(np.int16)
        
        return enhanced_signal

    def calculate_steering_delays(self, look_direction):
        """
        Calculate steering delays for beamforming toward a specific direction
        """
        # Normalize look direction
        look_dir_norm = look_direction / np.linalg.norm(look_direction)
        
        delays = []
        for pos in self.array_positions:
            # Calculate time delay based on projected distance on look direction
            projection = np.dot(pos, look_dir_norm)
            delay = projection / self.speed_of_sound
            delays.append(delay)
        
        return np.array(delays)

    def localize_sound_source(self, multi_channel_audio):
        """
        Simple sound source localization using time difference of arrival
        """
        try:
            # Calculate cross-correlation between channels to find delays
            # For simplicity, just using first two channels
            ch1 = multi_channel_audio[:, 0]
            ch2 = multi_channel_audio[:, 1]
            
            # Calculate cross-correlation to find time delay
            correlation = signal.correlate(ch1, ch2, mode='full')
            lags = signal.correlation_lags(len(ch1), len(ch2), mode='full')
            
            # Find index of maximum correlation
            lag_idx = np.argmax(np.abs(correlation))
            lag_samples = lags[lag_idx]
            time_delay = lag_samples / self.sample_rate
            
            # Calculate angle based on known mic spacing and speed of sound
            # For a pair of microphones spaced dx apart, angle theta is:
            # sin(theta) = (delay * speed_of_sound) / dx
            dx = 0.04  # 40mm spacing
            angle_rad = np.arcsin((time_delay * self.speed_of_sound) / dx)
            
            # Convert to 3D direction vector
            direction_3d = np.array([
                np.cos(angle_rad),
                0.0,
                np.sin(angle_rad)
            ])
            
            return direction_3d
            
        except Exception as e:
            self.get_logger().warn(f"Sound source localization failed: {e}")
            return None

    def publish_source_location(self, direction):
        """Publish sound source location as a point cloud"""
        try:
            # Create a point cloud with the sound source direction
            cloud = PointCloud()
            cloud.header.stamp = self.get_clock().now().to_msg()
            cloud.header.frame_id = "base_link"  # Robot base frame
            
            point = Point32()
            # Scale direction for visualization (e.g., 1 meter unit vector)
            scale_factor = 1.0
            point.x = scale_factor * direction[0]
            point.y = scale_factor * direction[1]
            point.z = scale_factor * direction[2]
            
            cloud.points = [point]
            self.source_pub.publish(cloud)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing sound source location: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BeamformingNode()
    
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
1. Update the package's setup.py:

```python
from setuptools import find_packages, setup

package_name = 'audio_processing'

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
    description='Package for audio processing with microphone arrays',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_publisher = audio_processing.audio_publisher:main',
            'beamforming_node = audio_processing.beamforming:main',
        ],
    },
)
```

### 5. Build and Test
1. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select audio_processing
source install/setup.bash
```

2. Run the audio publisher:
```bash
ros2 run audio_processing audio_publisher
```

3. Run the beamforming node:
```bash
ros2 run audio_processing beamforming
```

## Configuration Files

### 1. Audio Configuration File
Create `audio_processing/config/audio_params.yaml`:

```yaml
# Audio Processing Parameters
audio_publisher:
  ros__parameters:
    sample_rate: 16000
    channels: 4
    chunk_size: 1024
    device_index: 0  # Index of audio device to use
    
beamforming_node:
  ros__parameters:
    sample_rate: 16000
    frame_size: 1024
    fft_size: 512
    overlap: 0.5
    speed_of_sound: 343.0
    # Array geometry (positions in meters relative to center)
    mic_positions: [
      [0.0, 0.0, 0.0],
      [0.04, 0.0, 0.0],
      [0.0, 0.04, 0.0],
      [-0.04, 0.0, 0.0]
    ]
```

## Testing and Validation

### 1. Audio Level Testing
1. Monitor audio levels:
```bash
# Record and play back to verify functionality
arecord -D hw:1,0 -f cd -d 10 test.wav
aplay test.wav
```

### 2. ROS 2 Topic Monitoring
```bash
# Monitor audio data
ros2 topic echo /audio_input

# Monitor processed audio
ros2 topic echo /audio_processed

# Monitor sound source localization
ros2 topic echo /sound_source_location
```

### 3. Audio Quality Metrics
1. Check for clipping, noise levels, and signal quality
2. Test directionality by speaking from different positions
3. Verify beamforming effectiveness in noisy environments

## Common Configurations

### 1. Far-Field Voice Capture
For capturing voice commands from across a room:
- Use circular array with 4-8 microphones
- Apply beamforming toward user position
- Use noise suppression algorithms
- Sample at 16kHz for Whisper compatibility

### 2. Near-Field Interaction
For close-range voice interaction:
- Use 2-microphone array for stereo capture
- Apply wind noise reduction
- Focus on front-directional pickup

### 3. Omnidirectional Capture
For capturing sounds from all directions:
- Use circular array without beamforming
- Enable all microphones equally
- Use for sound detection and classification

## Troubleshooting

### Common Issues
1. **No Audio Input**: Check device selection and permissions
2. **High Latency**: Reduce chunk size and optimize processing code
3. **Noise Issues**: Apply noise reduction and ensure proper grounding
4. **Channel Mismatch**: Verify channel count matches hardware

### Diagnostic Commands
```bash
# Check available audio devices
arecord -l

# Test specific device
arecord -D hw:CARD,DEV -f cd -d 5 test.wav

# Check for audio device permissions
groups $USER
# If not in audio group: sudo usermod -a -G audio $USER
```

## Performance Optimization

### 1. CPU Usage
- Use optimized FFT libraries
- Process in chunks rather than sample-by-sample
- Apply decimation if high sample rates aren't needed

### 2. Memory Usage
- Use efficient data structures
- Release buffers appropriately
- Consider memory pooling for real-time processing

### 3. Latency Management
- Use appropriate buffer sizes
- Process in separate threads if needed
- Consider using RT kernel for critical applications

## Integration with Voice Processing

### 1. Connection to Whisper
- Ensure 16kHz sampling rate to match Whisper requirements
- Provide clean, de-noised signal to ASR system
- Match audio format (typically 16-bit PCM)

### 2. Real-Time Processing
- Buffer audio for appropriate time windows
- Process continuously to catch wake words
- Minimize processing pipeline to reduce latency

## Security Considerations

### Privacy
- Encrypt audio streams if transmitting over network
- Anonymize data in cloud processing
- Follow privacy regulations (GDPR, CCPA, etc.)

### Malicious Audio
- Implement audio anomaly detection
- Validate audio input lengths
- Protect against adversarial audio attacks

## Maintenance

### Regular Checks
1. Verify microphone functioning with test recordings
2. Check for physical damage to cables and connectors
3. Monitor audio quality metrics over time
4. Update audio processing firmware if available