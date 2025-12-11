# Lab 4.1: Voice Command Processing with Whisper

## Overview
This lab provides hands-on experience with voice command processing using OpenAI's Whisper and integrating it with a robotic system. You'll create a system that receives voice commands, processes them with Whisper, and executes appropriate actions.

## Prerequisites
- ROS 2 Humble Hawksbill installed
- OpenAI Whisper installed
- Audio input device (microphone or microphone array)
- Basic understanding of voice processing concepts from Lesson 4.1

## Learning Objectives
By the end of this lab, students will be able to:
- Set up Whisper for voice processing in ROS 2
- Create a voice command processing pipeline
- Integrate voice processing with robot control
- Implement command validation and safety checks
- Process natural language robot commands

## Lab Duration
Estimated time: 4-5 hours

## Step-by-Step Instructions

### Step 1: Verify Whisper Installation
1. Check if Whisper is installed:
```bash
python3 -c "import whisper; print('Whisper version:', whisper.__version__)"
```

2. If not installed, install Whisper:
```bash
pip3 install openai-whisper
```

3. For GPU acceleration, ensure you have PyTorch with CUDA:
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Step 2: Install Additional Dependencies
1. Install audio processing libraries:
```bash
pip3 install pyaudio soundfile librosa
```

2. Install ROS 2 audio packages:
```bash
sudo apt install ros-humble-audio-common-msgs
```

### Step 3: Create the Whisper Node Package
1. Create a new ROS 2 package for voice processing:
```bash
cd ~/physical_ai_ws/src
ros2 pkg create --build-type ament_python voice_command_processing
```

2. Update the package.xml with dependencies:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>voice_command_processing</name>
  <version>0.0.0</version>
  <description>Package for voice command processing with Whisper</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>audio_common_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 4: Create the Whisper Node
1. In the package directory, create the main Whisper node file `voice_command_processing/whisper_node.py`:

```python
import rclpy
from rclpy.node import Node
import whisper
import torch
import numpy as np
import pyaudio
import wave
import threading
import queue
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
import json


class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        
        # Check for CUDA availability
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")
        
        # Load Whisper model (using 'tiny' for faster processing in lab)
        self.get_logger().info("Loading Whisper model...")
        self.model = whisper.load_model("tiny", device=self.device)
        self.get_logger().info("Whisper model loaded successfully")
        
        # Initialize audio processing
        self.audio_queue = queue.Queue()
        self.recording = False
        self.sample_rate = 16000  # Standard for Whisper
        self.chunk_size = 1024
        
        # Setup subscribers and publishers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )
        
        self.transcription_pub = self.create_publisher(
            String,
            'transcribed_text',
            10
        )
        
        self.command_pub = self.create_publisher(
            String,
            'parsed_command',
            10
        )
        
        self.recording_status_pub = self.create_publisher(
            Bool,
            'recording_status',
            10
        )
        
        # Setup recording service
        self.start_recording_service = self.create_service(
            Bool,
            'start_voice_recording',
            self.start_recording_callback
        )
        
        self.stop_recording_service = self.create_service(
            Bool,
            'stop_voice_recording',
            self.stop_recording_callback
        )
        
        # Start audio processing thread
        self.audio_thread = threading.Thread(target=self.process_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.get_logger().info("Whisper Node initialized")

    def audio_callback(self, msg):
        """Callback for audio input from ROS topic"""
        # Convert byte data to numpy array
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        self.audio_queue.put(audio_data)

    def start_recording_callback(self, request, response):
        """Start voice recording service"""
        self.recording = True
        status_msg = Bool()
        status_msg.data = True
        self.recording_status_pub.publish(status_msg)
        self.get_logger().info("Voice recording started")
        response.data = True
        return response

    def stop_recording_callback(self, request, response):
        """Stop voice recording service"""
        self.recording = False
        status_msg = Bool()
        status_msg.data = False
        self.recording_status_pub.publish(status_msg)
        self.get_logger().info("Voice recording stopped")
        response.data = True
        return response

    def process_audio(self):
        """Process audio chunks in a separate thread"""
        audio_buffer = np.array([])
        
        while rclpy.ok():
            try:
                # Get audio chunk from queue
                chunk = self.audio_queue.get(timeout=0.1)
                
                if self.recording:
                    # Add chunk to buffer
                    audio_buffer = np.concatenate([audio_buffer, chunk])
                    
                    # If buffer is large enough, transcribe
                    if len(audio_buffer) >= self.sample_rate * 2:  # At least 2 seconds
                        # Transcribe the audio
                        transcription = self.transcribe_audio(audio_buffer)
                        
                        if transcription.strip():  # If transcription is not empty
                            # Publish transcribed text
                            transcribed_msg = String()
                            transcribed_msg.data = transcription
                            self.transcription_pub.publish(transcribed_msg)
                            
                            # Parse and publish command
                            command = self.parse_command(transcription)
                            if command:
                                command_msg = String()
                                command_msg.data = command
                                self.command_pub.publish(command_msg)
                                self.get_logger().info(f'Command: {command}')
                        
                        # Reset buffer after transcription
                        audio_buffer = np.array([])
            except queue.Empty:
                continue  # Continue if queue is empty

    def transcribe_audio(self, audio_data):
        """Transcribe audio using Whisper"""
        try:
            # Convert to 16kHz if needed
            if len(audio_data) == 0:
                return ""
                
            # Pad if too short (Whisper expects at least 0.1 seconds)
            if len(audio_data) < self.sample_rate * 0.1:
                padding = int(self.sample_rate * 0.1) - len(audio_data)
                audio_data = np.pad(audio_data, (0, padding), mode='constant')
            
            # Transcribe using Whisper
            result = self.model.transcribe(audio_data, fp16=False if self.device == 'cpu' else True)
            return result['text'].strip()
        except Exception as e:
            self.get_logger().error(f"Error in transcription: {e}")
            return ""

    def parse_command(self, text):
        """Parse natural language command and convert to robot command"""
        text = text.lower().strip()
        
        # Define simple command patterns
        if "move forward" in text or "go forward" in text or "move ahead" in text:
            return "MOVE_FORWARD"
        elif "move backward" in text or "go back" in text or "reverse" in text:
            return "MOVE_BACKWARD"
        elif "turn left" in text or "rotate left" in text:
            return "TURN_LEFT"
        elif "turn right" in text or "rotate right" in text:
            return "TURN_RIGHT"
        elif "stop" in text or "halt" in text:
            return "STOP"
        elif "pick up" in text or "grasp" in text or "grab" in text:
            return "PICK_UP_OBJECT"
        elif "put down" in text or "place" in text or "release" in text:
            return "PUT_DOWN_OBJECT"
        elif "come to me" in text or "follow me" in text:
            return "FOLLOW_ME"
        elif "go to" in text or "navigate to" in text:
            # Extract location if possible
            if "kitchen" in text:
                return "NAVIGATE_TO_KITCHEN"
            elif "living room" in text:
                return "NAVIGATE_TO_LIVING_ROOM"
            elif "bedroom" in text:
                return "NAVIGATE_TO_BEDROOM"
            else:
                return "NAVIGATE_TO_LOCATION"
        else:
            # Return the original text if no recognized command
            return f"UNKNOWN_COMMAND: {text}"

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.recording = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    
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

### Step 5: Create Audio Input Node
1. Create an audio input node `voice_command_processing/audio_input_node.py`:

```python
import rclpy
from rclpy.node import Node
import pyaudio
import numpy as np
from audio_common_msgs.msg import AudioData


class AudioInputNode(Node):
    def __init__(self):
        super().__init__('audio_input_node')
        
        # Setup audio parameters
        self.rate = 16000  # Whisper standard
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        
        # Setup publisher
        self.publisher = self.create_publisher(AudioData, 'audio_input', 10)
        
        # Setup PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Start audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        # Timer to periodically publish audio data
        self.timer = self.create_timer(0.1, self.publish_audio)  # 100ms interval
        
        self.get_logger().info("Audio Input Node initialized")

    def publish_audio(self):
        """Read audio data and publish to topic"""
        try:
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            
            # Create and publish audio message
            msg = AudioData()
            msg.data = data
            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error reading audio: {e}")

    def destroy_node(self):
        """Clean up when node is destroyed"""
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioInputNode()
    
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

### Step 6: Create Voice Command Parser Node
1. Create a command parsing node `voice_command_processing/command_parser_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')
        
        # Setup subscriber for parsed commands
        self.command_sub = self.create_subscription(
            String,
            'parsed_command',
            self.command_callback,
            10
        )
        
        # Setup publisher for robot control commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info("Command Parser Node initialized")

    def command_callback(self, msg):
        """Process parsed commands and generate robot control commands"""
        command = msg.data.upper()
        self.get_logger().info(f"Processing command: {command}")
        
        # Create Twist message for robot movement
        twist = Twist()
        
        if command == "MOVE_FORWARD":
            twist.linear.x = 0.5  # Move forward at 0.5 m/s
        elif command == "MOVE_BACKWARD":
            twist.linear.x = -0.5  # Move backward at 0.5 m/s
        elif command == "TURN_LEFT":
            twist.angular.z = 0.5  # Turn left at 0.5 rad/s
        elif command == "TURN_RIGHT":
            twist.angular.z = -0.5  # Turn right at 0.5 rad/s
        elif command == "STOP":
            # Twist is already zero, so robot will stop
            pass
        else:
            self.get_logger().info(f"Command not mapped to robot action: {command}")
            return  # Don't publish if command isn't mapped to movement
        
        # Publish the command
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Published command: linear.x={twist.linear.x}, angular.z={twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = CommandParserNode()
    
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

### Step 7: Update setup.py
1. Update the package's `setup.py` to include entry points:

```python
from setuptools import find_packages, setup

package_name = 'voice_command_processing'

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
    description='Package for voice command processing with Whisper',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_node = voice_command_processing.whisper_node:main',
            'audio_input_node = voice_command_processing.audio_input_node:main',
            'command_parser_node = voice_command_processing.command_parser_node:main',
        ],
    },
)
```

### Step 8: Build and Test
1. Build the workspace:
```bash
cd ~/physical_ai_ws
colcon build --packages-select voice_command_processing
```

2. Source the workspace:
```bash
source install/setup.bash
```

3. Test the audio input node separately first:
```bash
ros2 run voice_command_processing audio_input_node
```

4. In a new terminal, run the Whisper node:
```bash
source install/setup.bash
ros2 run voice_command_processing whisper_node
```

5. In another terminal, run the command parser node:
```bash
source install/setup.bash
ros2 run voice_command_processing command_parser_node
```

6. Test by publishing voice commands or using services to control recording.

### Step 9: Start Voice Recording
1. Use the service to start recording:
```bash
ros2 service call /start_voice_recording std_msgs/srv/Bool "{data: true}"
```

2. Speak commands to your microphone.

3. Use the service to stop recording when done:
```bash
ros2 service call /stop_voice_recording std_msgs/srv/Bool "{data: false}"
```

### Step 10: Monitor Output
Monitor the transcribed text and parsed commands:
```bash
# Monitor transcribed text
ros2 topic echo /transcribed_text

# Monitor parsed commands
ros2 topic echo /parsed_command

# Monitor robot commands (if connected to a robot)
ros2 topic echo /cmd_vel
```

### Step 11: Experiment and Extend
1. Modify the command parsing logic to recognize more commands
2. Add safety checks to prevent dangerous commands
3. Improve the audio preprocessing for better quality
4. Adjust the Whisper model (try 'base' for better accuracy)
5. Add confidence scoring to filter uncertain transcriptions

## Deliverable
Submit a report containing:
1. Screenshots of the running voice command system
2. Examples of voice commands and their interpretations
3. Modified code that demonstrates at least one extension from Step 11
4. Analysis of system accuracy and performance
5. Video demonstration of voice-controlled robot operation
6. Discussion of challenges encountered and solutions implemented

## Assessment Criteria
- Whisper integration with ROS 2 (25%)
- Voice command processing pipeline (25%)
- Command parsing and robot control (20%)
- System robustness and error handling (15%)
- Report quality and analysis (15%)

## Troubleshooting
- **Audio device not found**: Verify microphone is connected and accessible
- **CUDA errors**: Ensure PyTorch with CUDA is installed
- **Whisper loading errors**: Check internet connection for model download
- **High latency**: Use smaller Whisper model or optimize audio processing
- **Recognition issues**: Ensure quiet environment and clear speech

## Additional Resources
- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [ROS 2 Audio Processing](https://index.ros.org/p/audio_common/)
- [PyAudio Documentation](https://pyaudio.readthedocs.io/)