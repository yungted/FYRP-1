# Robert Robot - LLM-Controlled Navigation System
## Complete Setup and Usage Guide

---

## 📋 System Overview

This system enables natural language control of a robot running ROS 2 Humble on a Jetson Orin Nano. The robot understands high-level commands ("go to kitchen") and converts them to low-level ROS commands with safety constraints.

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     USER (Voice Input)                      │
│                    via Deepgram + TTS                       │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│              PC Side (LLM Interface)                         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ - Deepgram: Speech Recognition & Wake Word          │  │
│  │ - Ollama (GPT-OSS): Context-aware Command Parsing   │  │
│  │ - Waypoint Manager: Store & Track Locations          │  │
│  │ - TTS: Audio Feedback                                │  │
│  └──────────────────────────────────────────────────────┘  │
│                        ↓ MQTT                               │
└────────────────────┬────────────────────────────────────────┘
                     │
        ┌────────────┴──────────────┐
        │      MQTT Broker          │
        │   192.168.0.165:1883      │
        └────────────┬──────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│          Jetson Orin Nano (Robot Side)                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ MQTT-ROS Bridge                                      │  │
│  │ - Low-level: Twist commands → /cmd_vel              │  │
│  │ - High-level: NavigateToPose → Nav2 Stack           │  │
│  │ - Feedback: Pose updates, Status messages            │  │
│  └──────────────────────────────────────────────────────┘  │
│                        ↓ ROS                                │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ ROS 2 Humble Components                              │  │
│  │ - Differential Drive Base Controller                 │  │
│  │ - AMCL: Localization                                 │  │
│  │ - Nav2: Autonomous Navigation                        │  │
│  └──────────────────────────────────────────────────────┘  │
│                        ↓ Hardware                           │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ - DC Motors (with encoders)                          │  │
│  │ - Lidar/Sensors                                      │  │
│  │ - IMU                                                │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚀 Installation & Setup

### Prerequisites
- Jetson Orin Nano running Ubuntu 22.04 with ROS 2 Humble
- PC/Laptop with Python 3.8+
- MQTT Broker (Mosquitto) running on network at `192.168.0.165:1883`
- Network connectivity between PC and Jetson (WiFi or Ethernet)

### PC Side Setup

#### 1. Install Required Python Packages

```bash
pip install paho-mqtt websockets numpy sounddevice opencv-python pillow
pip install TTS ollama moondream
```

#### 2. Install Deepgram SDK (Optional - for advanced features)
```bash
pip install deepgram-sdk
```

#### 3. Place Files

Copy these files to your project directory:
- `LLM Interface with ROS output for Pi.py` - Main PC controller
- `waypoint_manager.py` - Waypoint management module
- `devices.json` - Device registry (auto-generated)
- `waypoints.json` - Saved waypoints (auto-generated)

#### 4. Configure API Keys

Edit the top of `LLM Interface with ROS output for Pi.py`:

```python
DEEPGRAM_KEY = "your_deepgram_key"        # From deepgram.com
OLLAMA_KEY = "your_ollama_key"            # From ollama provider
MOONDREAM_KEY = "your_moondream_key"      # From moondream.ai
MQTT_BROKER = "192.168.0.165"             # Your network MQTT broker
```

#### 5. Test PC Setup

```bash
cd /path/to/fyrp
python3 "LLM Interface with ROS output for Pi.py"
```

You should see:
```
✅ TTS Model Loaded.
✅ MQTT connected (192.168.0.165) rc=0
🎙️ Connecting to Deepgram...
✅ Listening... Say 'Hey Robert'!
```

---

### Jetson Side Setup

#### 1. Install ROS Dependencies

```bash
sudo apt-get update
sudo apt-get install python3-paho-mqtt python3-nav2-simple-commander
```

#### 2. Setup ROS Packages

Ensure these packages are installed:
```bash
ros2 pkg find geometry_msgs
ros2 pkg find nav2_msgs
ros2 pkg find nav_msgs
```

If missing, install navigation2:
```bash
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup
```

#### 3. Place Bridge Script

Copy `mqtt_ros_bridge_ENHANCED.py` to the Jetson:

```bash
scp mqtt_ros_bridge_ENHANCED.py jetson@jetson-ip:~/
```

#### 4. Create Launch File (Optional but Recommended)

Create `~/launch_bridge.sh`:

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Start MQTT-ROS bridge
python3 ~/mqtt_ros_bridge_ENHANCED.py
```

Make executable:
```bash
chmod +x ~/launch_bridge.sh
```

#### 5. Test Jetson Setup

```bash
# Terminal 1: Start Nav2
ros2 launch nav2_bringup navigation_launch.py map:=/path/to/map.yaml use_sim_time:=false

# Terminal 2: Start MQTT Bridge
python3 mqtt_ros_bridge_ENHANCED.py
```

You should see:
```
[mqtt_ros_bridge]: 🤖 MQTT-ROS Bridge initialized successfully
[mqtt_ros_bridge]: 📡 MQTT connected (rc=0)
```

---

## 💬 Usage Examples

### Basic Voice Commands

#### Local Movement (Low-level)
```
User: "Hey Robert, move forward"
→ Robot moves forward ~0.6m at safe speed
→ TTS: "Moving forward."

User: "Hey Robert, move forward 2 meters"
→ Robot calculates duration for 2m at 0.3 m/s
→ TTS: "Moving forward 2 meters."

User: "Hey Robert, turn left 90 degrees"
→ Robot rotates left 90°
→ TTS: "Turning left 90 degrees."

User: "Hey Robert, come here slowly"
→ Robot uses 'approach' command (slow, steady)
→ TTS: "Approaching you carefully."
```

#### Waypoint Navigation (High-level)
```
User: "Hey Robert, save this location as kitchen"
→ Robot records current AMCL pose with name 'kitchen'
→ Waypoint saved to waypoints.json
→ TTS: "Saved this location as kitchen. I can navigate here later if you ask."

User: "Hey Robert, go to kitchen"
→ LLM interprets intent, sends Nav2 goal
→ Robot autonomously navigates using path planning
→ Avoids obstacles, uses local costmaps
→ TTS: "I'm navigating to the kitchen. This may take a moment."
→ When reached: TTS: "Reached the kitchen"

User: "Hey Robert, what locations have you saved?"
→ LLM lists all waypoints
→ TTS: "Here are the locations I have saved: kitchen: Main kitchen area, office: Office location, ..."
```

#### Complex Sequences
```
User: "Hey Robert, turn around and go to the bedroom"
→ LLM creates multi-step sequence:
   1. Execute ros_command: rotate_180
   2. Execute navigate_to_waypoint: bedroom
→ TTS: "Turning around and heading to the bedroom."

User: "Hey Robert, move forward 1 meter then turn left"
→ LLM creates sequence:
   1. Execute ros_command: move_forward (distance: 1.0)
   2. Execute ros_command: turn_left
```

#### Safety Commands
```
User: "Hey Robert, stop!" / "Freeze!" / "Emergency!"
→ Robot immediately halts (emergency_stop)
→ TTS: "Stopping immediately."

User: "Hey Robert, go really fast"
→ Robot politely refuses
→ TTS: "I'm sorry, I can only move at safe speeds for everyone's safety. 
         Would you like me to move forward at my normal pace?"
```

---

## 🎯 Command Types & Syntax

### ROS Commands (Low-level Movement)

Available commands (hardcoded safe velocities):

```
move_forward       - Move forward at 0.3 m/s (default 3 seconds ~ 0.9m)
move_backward      - Move backward at 0.2 m/s (default 3 seconds ~ 0.6m)
turn_left          - Rotate left at 0.5 rad/s (default 2 seconds ~ 45°)
turn_right         - Rotate right at 0.5 rad/s (default 2 seconds ~ 45°)
stop               - Immediately halt (duration 0)
slow_forward       - Slow approach at 0.15 m/s (default 4 seconds ~ 0.6m)
rotate_180         - Full rotation (2 seconds at 0.5 rad/s)
slight_left        - Forward+left at 0.2m/s + 0.2rad/s
slight_right       - Forward+right at 0.2m/s - 0.2rad/s
approach           - Careful approach at 0.2 m/s (2.5 seconds)
retreat            - Back away at 0.15 m/s (2.5 seconds)
guide_forward      - Following/guidance at 0.25 m/s (4 seconds)
```

### Modular Parameters

You can modify commands with parameters:

```json
{
  "mcp_action": "ros_command",
  "parameters": [{
    "command": "move_forward",
    "distance": 2.5        // Move 2.5 meters (auto-calculates duration)
  }]
}
```

Or with duration:

```json
{
  "mcp_action": "ros_command",
  "parameters": [{
    "command": "move_forward",
    "duration": 5.0        // Move for 5 seconds
  }]
}
```

Or with angle:

```json
{
  "mcp_action": "ros_command",
  "parameters": [{
    "command": "turn_left",
    "angle": 45            // Turn 45 degrees
  }]
}
```

### Waypoint Actions

```json
{
  "mcp_action": "navigate_to_waypoint",
  "parameters": [{
    "waypoint": "kitchen"   // Navigate to saved waypoint
  }]
}
```

```json
{
  "mcp_action": "save_waypoint",
  "parameters": [{
    "waypoint_name": "office",
    "description": "My office room"
  }]
}
```

---

## ⚙️ Configuration & Tuning

### PC-Side Calibration (`LLM Interface with ROS output for Pi.py`)

Adjust these values if robot movements aren't accurate:

```python
# If robot moves LESS than expected, INCREASE these values
# If robot moves MORE than expected, DECREASE these values

DISTANCE_CALIBRATION = 1.0   # Default: 1.0 (try 1.5, 2.0 if under-moving)
DURATION_CALIBRATION = 1.0   # Default: 1.0 (time multiplier)
ANGLE_CALIBRATION = 1.0      # Default: 1.0 (rotation multiplier)

# Measured actual velocities (update with real measurements)
BASE_LINEAR_VELOCITY = 0.3   # m/s - Your robot's actual forward speed
BASE_ANGULAR_VELOCITY = 0.5  # rad/s - Your robot's actual rotation speed
SLOW_LINEAR_VELOCITY = 0.15  # m/s - Slow movement speed

# Add extra time for acceleration/deceleration
ACCEL_COMPENSATION_SEC = 0.3  # seconds
```

### Velocity Limits

Maximum safe velocities (prevent damage/injury):

```python
MAX_LINEAR_VELOCITY = 0.5    # m/s - Absolute maximum forward speed
MAX_ANGULAR_VELOCITY = 0.8   # rad/s - Absolute maximum rotation
MAX_DISTANCE_METERS = 5.0    # Single command limit
MAX_DURATION_SECONDS = 15.0  # Single command time limit
MAX_ANGLE_DEGREES = 360.0    # Single command rotation limit
```

### Jetson-Side Configuration

Edit `mqtt_ros_bridge_ENHANCED.py`:

```python
MQTT_BROKER = "192.168.0.165"
CMD_VEL_TOPIC = "/diffbot_base_controller/cmd_vel_unstamped"
AMCL_POSE_TOPIC = "/amcl_pose"

# Navigation timeout (5 minutes)
NAV_TIMEOUT_SECONDS = 300

# Velocity publish rate (10 Hz = every 0.1 seconds)
CMD_VEL_PUBLISH_RATE = 10
```

---

## 🐛 Troubleshooting

### PC Side Issues

#### "❌ MQTT Connection Failed"
```
Problem: PC can't reach MQTT broker
Solution:
1. Check MQTT_BROKER IP: ping 192.168.0.165
2. Start MQTT broker: sudo systemctl start mosquitto
3. Check firewall: sudo ufw allow 1883
```

#### "⚠️ Failed to parse extracted JSON from LLM"
```
Problem: LLM response isn't valid JSON
Solution:
1. Check Ollama connection: curl http://localhost:11434
2. Update system prompt (may need more specific formatting)
3. Add error logging to ask_ollama() function
```

#### "🎙️ Deepgram Connection Failed"
```
Problem: Can't connect to Deepgram API
Solution:
1. Verify DEEPGRAM_KEY is valid
2. Check internet connection: ping api.deepgram.com
3. Check rate limits (free tier has limits)
```

#### "⏳ TTS Model Loaded hangs"
```
Problem: TTS initialization takes too long
Solution:
1. Pre-download model: python3 -m TTS.cli --model_name tts_models/en/ljspeech/vits
2. Check disk space
3. Use GPU: CUDA_VISIBLE_DEVICES=0 python3 script.py
```

### Jetson Side Issues

#### "❌ MQTT Connection Failed"
```
Problem: Jetson can't reach MQTT broker
Solution:
1. Check IP: ip addr (find your IP)
2. Check PC MQTT running: sudo systemctl status mosquitto
3. Check firewall on PC: sudo ufw status
```

#### "⏳ Waiting for Nav2 action server"
```
Problem: Nav2 not started or not responding
Solution:
1. Verify Nav2 is running: ros2 action list | grep navigate
2. Check ROS setup: source ~/ros2_ws/install/setup.bash
3. Restart nav2: ros2 launch nav2_bringup navigation_launch.py
```

#### "❌ Navigation failed: RuntimeError"
```
Problem: Nav2 navigation goal rejected
Solution:
1. Check map file exists: ls /path/to/map.yaml
2. Initialize robot pose in RVIZ
3. Check costmaps: ros2 service call /global_costmap/get_costmap_updates
4. Verify goal is on map (not outside bounds)
```

#### "⏹️ Movement doesn't stop"
```
Problem: Robot keeps moving after command
Solution:
1. Check /cmd_vel topic is receiving zero values: ros2 topic echo /cmd_vel
2. Verify motor controller is listening to /cmd_vel
3. Check ROS_DOMAIN_ID matches on all systems
4. Manually publish stop: ros2 topic pub -1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}"
```

### Network Issues

#### MQTT Connection Drops
```
Solutions:
1. Keep-alive: MQTT_TIMEOUT = 60 (in bridge code)
2. Check WiFi signal strength
3. Reduce MQTT_PUBLISH_RATE if too aggressive
4. Check broker resources: mosquitto -v
```

#### High Latency/Lag
```
Problem: Commands execute slowly
Solution:
1. Check network: ping jetson-ip (should be <100ms)
2. Monitor network traffic
3. Reduce Deepgram/Ollama API calls (local fallbacks)
4. Use wired Ethernet instead of WiFi
```

---

## 📊 File Structure

```
~/FYRP 1 Main Github Codes/
├── LLM Interface with ROS output for Pi.py    [MAIN PC CONTROLLER]
├── mqtt_ros_bridge_ENHANCED.py                 [JETSON BRIDGE]
├── waypoint_manager.py                         [WAYPOINT MANAGEMENT]
├── devices.json                                [AUTO-GENERATED: Device registry]
├── waypoints.json                              [AUTO-GENERATED: Saved locations]
├── Waypoint_test.py                            [Original waypoint test]
├── mqtt_ros_bridge_FIXED.py                    [Original bridge]
└── [Other device control scripts...]
```

---

## 🔒 Safety Features

### Built-In Safeguards

1. **Velocity Limiting**
   - All commands clamped to MAX_LINEAR_VELOCITY (0.5 m/s)
   - All rotations clamped to MAX_ANGULAR_VELOCITY (0.8 rad/s)
   - Prevents dangerous high-speed operation

2. **Command Whitelisting**
   - Only predefined safe commands allowed
   - LLM cannot invent new commands
   - All velocities hardcoded (not controllable by LLM)

3. **Emergency Stop**
   - Immediate halt on user command: "Stop!", "Emergency!", "Freeze!"
   - Emergency stop flag prevents new commands
   - Continuous zero-velocity publishing ensures motor stop

4. **Duration Limits**
   - Max 15 seconds per command (prevents runaway)
   - Max 5 meters per movement
   - Max 360° rotation per command

5. **Context Awareness**
   - LLM understands current pose and navigation status
   - Won't send contradictory commands
   - Asks for clarification if ambiguous

6. **Intent Analysis**
   - High-level command interpretation
   - Safety-first reasoning before execution
   - Refuses dangerous requests politely

---

## 📈 Performance Optimization

### PC Side
- Pre-load TTS model on startup
- Cache device registry locally
- Batch MQTT subscriptions
- Use threading for non-blocking operations

### Jetson Side
- Nav2 maps loaded to RAM
- Use local AMCL instead of GPS
- Continuous velocity publishing (not single commands)
- Efficient quaternion normalization

### Network
- MQTT QoS=1 for reliability
- Retained messages for state sync
- Compressed JSON payloads
- Minimal topic duplication

---

## 🧪 Testing Checklist

- [ ] **Connection Test**
  ```bash
  mosquitto_sub -h 192.168.0.165 -t "robot/#"  # On PC
  # Publish from Jetson: should see messages
  ```

- [ ] **Deepgram Test**
  ```python
  python3 -c "import websockets; print('OK')"
  ```

- [ ] **Ollama Test**
  ```bash
  curl http://localhost:11434/api/tags
  ```

- [ ] **ROS2 Bridge Test**
  ```bash
  # Check topics
  ros2 topic list | grep -E "(cmd_vel|amcl_pose)"
  ```

- [ ] **Waypoint Persistence**
  ```bash
  # Check file exists and is valid JSON
  python3 -m json.tool waypoints.json
  ```

- [ ] **Full Integration Test**
  1. Start MQTT broker
  2. Start Nav2 on Jetson
  3. Start Bridge on Jetson
  4. Start PC LLM interface
  5. Say "Hey Robert, move forward"
  6. Verify: Robot moves, feedback returned, TTS speaks

---

## 📚 Additional Resources

- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **Nav2 Documentation**: https://navigation.ros.org/
- **Deepgram API**: https://developers.deepgram.com/
- **MQTT Protocol**: https://mqtt.org/
- **Ollama**: https://ollama.ai/

---

## 🤝 Contributing & Customization

### Adding New Waypoints Programmatically
```python
from waypoint_manager import WaypointManager

mgr = WaypointManager()
mgr.add_waypoint("conference_room", 5.0, 3.5, 0.0, 1.0, "Large conference table")
mgr.save_waypoints()
```

### Adding New ROS Commands
```python
# Add to SAFE_ROS_COMMANDS dictionary
SAFE_ROS_COMMANDS["custom_move"] = {
    "linear_x": 0.25,
    "angular_z": 0.0,
    "duration": 3.0,
    "type": "linear"
}
```

### Custom Intents
```python
# Add to system prompt in ask_ollama()
"CUSTOM INTENT: 'go dancing' → {\"command\": \"slight_left\"}, {\"command\": \"slight_right\"} (alternating)"
```

---

## 📝 Changelog

### v2.0 (Enhanced)
- ✅ Added waypoint management system
- ✅ Context-aware LLM command interpretation
- ✅ Nav2 integration for autonomous navigation
- ✅ Current pose tracking and publication
- ✅ Comprehensive error handling
- ✅ Detailed feedback system
- ✅ Enhanced safety features
- ✅ Full documentation

### v1.0 (Original)
- Basic ROS command execution
- Deepgram voice recognition
- Ollama LLM integration
- MQTT messaging

---

## 📞 Support

For issues or questions:
1. Check the Troubleshooting section
2. Review system logs: `ros2 run rqt_console rqt_console`
3. Test components individually (MQTT, ROS, LLM separately)
4. Verify network connectivity between systems

---

**Last Updated**: January 2026  
**System**: Jetson Orin Nano + ROS 2 Humble + Ollama LLM
