# Quick Reference Guide
## Robert Robot Control System

---

## 🎯 Quick Start (5 minutes)

### 1. Start MQTT Broker (PC)
```bash
sudo systemctl start mosquitto
sudo systemctl status mosquitto
```

### 2. Start Jetson Services
```bash
# Terminal 1: Start Nav2
ros2 launch nav2_bringup navigation_launch.py map:=/path/to/map.yaml

# Terminal 2: Start MQTT-ROS Bridge
python3 mqtt_ros_bridge_ENHANCED.py
```

### 3. Start PC LLM Controller
```bash
python3 "LLM Interface with ROS output for Pi.py"
```

### 4. Test It!
Say: **"Hey Robert, move forward"**

---

## 🗣️ Voice Commands Cheat Sheet

### Movement Commands
| Command | Robot Action |
|---------|--------------|
| "move forward" | Move ~0.6m forward |
| "move backward" | Move ~0.4m backward |
| "turn left" | Rotate ~45° left |
| "turn right" | Rotate ~45° right |
| "move forward 2 meters" | Move exactly 2m forward |
| "turn left 90 degrees" | Rotate exactly 90° left |
| "approach me" | Slow careful approach |
| "back away" | Slow retreat |
| "turn around" | Full 180° rotation |
| "stop" | Emergency stop |

### Waypoint Commands
| Command | Robot Action |
|---------|--------------|
| "save this as kitchen" | Save current location as waypoint |
| "go to kitchen" | Navigate to saved waypoint |
| "what locations are saved" | List all waypoints |
| "navigate to bedroom" | Go to bedroom waypoint |

### Complex Commands
| Command | Robot Action |
|---------|--------------|
| "turn around and go to kitchen" | Multi-step: rotate + navigate |
| "move forward 1 meter then turn left" | Multi-step: move + turn |
| "come here slowly" | Use approach command |

---

## ⚙️ Configuration Cheat Sheet

### Velocity Limits (Don't change unless needed)
```python
MAX_LINEAR_VELOCITY = 0.5      # Max forward speed (m/s)
MAX_ANGULAR_VELOCITY = 0.8     # Max rotation speed (rad/s)
```

### Calibration (If robot moves wrong distance)
```python
# If robot moves TOO SHORT, INCREASE these
DISTANCE_CALIBRATION = 1.0     # Try: 1.5, 2.0
DURATION_CALIBRATION = 1.0     # Try: 1.2, 1.5

# If robot moves TOO FAR, DECREASE these
DISTANCE_CALIBRATION = 0.75    # Try: 0.5, 0.75
```

### Actual Robot Speeds (Measure and update)
```python
BASE_LINEAR_VELOCITY = 0.3     # Your actual forward speed
BASE_ANGULAR_VELOCITY = 0.5    # Your actual rotation speed
```

---

## 🐛 Quick Troubleshooting

### Robot doesn't respond
```bash
# 1. Check MQTT running
mosquitto_sub -h 192.168.0.165 -t "robot/#"

# 2. Check bridge running on Jetson
ps aux | grep mqtt_ros_bridge

# 3. Check network
ping 192.168.0.165
```

### Wrong movement distance
```python
# Measure actual movement, then adjust calibration:
# Expected 1m but moved 0.5m → Set DISTANCE_CALIBRATION = 2.0
```

### "Wake word not detected"
- Make sure you say one of:
  - "Hey Robert"
  - "Hi Robert"
  - "Okay Robert"
  - "Hey, Robert"

### Navigation fails
```bash
# Check Nav2 running
ros2 action list | grep navigate

# Check AMCL pose is initialized in RVIZ
# (Set initial pose on map)
```

### MQTT connection fails
```bash
# Check broker
mosquitto_sub -h 192.168.0.165 -t test

# Check firewall
sudo ufw allow 1883
```

---

## 📊 File Locations

| File | Purpose | PC/Jetson |
|------|---------|-----------|
| `LLM Interface with ROS output for Pi.py` | Main controller | PC |
| `waypoint_manager.py` | Waypoint storage | PC |
| `mqtt_ros_bridge_ENHANCED.py` | ROS bridge | Jetson |
| `waypoints.json` | Saved locations | PC |
| `devices.json` | Device registry | PC |

---

## 🔄 Data Flow

```
User Voice Input
    ↓
Deepgram (Speech → Text)
    ↓
Ollama LLM (Intent → Commands)
    ↓
MQTT Message
    ↓
Jetson MQTT-ROS Bridge
    ↓
ROS 2 (Nav2 or Velocity Control)
    ↓
Robot Motors
    ↓
Sensors (AMCL, Odometry)
    ↓
Feedback to PC
    ↓
TTS Response to User
```

---

## 📝 Waypoint JSON Format

```json
{
  "waypoints": {
    "kitchen": {
      "x": 1.97,
      "y": 12.24,
      "z": 1.0,
      "w": 0.0,
      "description": "Main kitchen area",
      "created_at": "2026-01-24T10:30:00",
      "visited_count": 5,
      "last_visited": "2026-01-24T14:22:00"
    }
  },
  "metadata": {
    "last_updated": "2026-01-24T14:22:00",
    "total_waypoints": 1
  }
}
```

---

## 🚨 Safety Rules

1. **Speed is LIMITED** - Robot can't go fast
2. **Distance is LIMITED** - Max 5m per command
3. **Time is LIMITED** - Max 15 seconds per command
4. **Commands are WHITELISTED** - Only predefined safe commands
5. **Velocities are HARDCODED** - LLM can't change them
6. **Emergency stop works** - Say "stop" anytime
7. **Context aware** - Robot knows where it is

---

## 🔧 Emergency Procedures

### Robot Won't Stop
```bash
# Terminal: Kill the bridge
pkill -f mqtt_ros_bridge

# Then: Manual stop via ROS
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}"
```

### MQTT Stuck
```bash
# Restart broker
sudo systemctl restart mosquitto

# Clear retained messages
mosquitto_pub -h 192.168.0.165 -t "robot/#" -r -n
```

### LLM Misunderstands
```python
# Add clarification to system prompt or rephrase:
# Instead of: "Go left"
# Say: "Turn left then move forward"
```

### Navigation Stuck
```bash
# Cancel current goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}"

# Re-initialize pose in RVIZ
# (Use 2D Pose Estimate tool)
```

---

## 📈 Common Adjustments

### Robot Moves Too Slow
```python
BASE_LINEAR_VELOCITY = 0.4  # Increase from 0.3
BASE_ANGULAR_VELOCITY = 0.7  # Increase from 0.5
```

### Robot Overshoots Distance
```python
DISTANCE_CALIBRATION = 0.8  # Decrease from 1.0
```

### Commands Too Short/Long
```python
# Adjust command defaults
SAFE_ROS_COMMANDS["move_forward"]["duration"] = 5.0  # Increase from 3.0
```

---

## 📞 Testing Commands

### MQTT Direct
```bash
# Publish movement command
mosquitto_pub -h 192.168.0.165 -t robot/ros_cmd -m '{
  "command": "move_forward",
  "twist": {"linear": {"x": 0.3, "y": 0.0, "z": 0.0}, "angular": {"z": 0.0}},
  "duration": 2.0
}'

# Monitor feedback
mosquitto_sub -h 192.168.0.165 -t "robot/ros_feedback"
```

### ROS Direct
```bash
# Publish velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.3}" -1

# Listen to pose
ros2 topic echo /amcl_pose
```

---

## 🎓 Advanced Usage

### Multi-step Sequences
```
User: "Go to kitchen, then turn around, then move forward 1 meter"
LLM generates:
1. navigate_to_waypoint(kitchen)
2. ros_command(rotate_180)
3. ros_command(move_forward, distance=1.0)
```

### Conditional Navigation
```
User: "Check if the kitchen is nearby, if not go there"
LLM:
1. Analyzes current pose
2. Checks waypoint distance
3. Decides best action
```

### Context-Aware Responses
```
User: "Come here"
LLM understands:
- Not "move forward" (no distance specified)
- Not "approach" (too gentle)
- Correct: "approach" command (safe, appropriate)
```

---

## 📚 Documentation Files

- **SETUP_AND_USAGE_GUIDE.md** - Detailed setup instructions
- **system_test.py** - Diagnostic testing tool
- **waypoint_manager.py** - Waypoint module source
- **mqtt_ros_bridge_ENHANCED.py** - Jetson bridge source

---

## 💡 Pro Tips

1. **Calibrate on flat ground** for accurate movements
2. **Use waypoints for repeating locations** instead of verbal coordinates
3. **Test MQTT first** before full system integration
4. **Monitor ROS logs** while testing: `ros2 run rqt_console rqt_console`
5. **Name waypoints clearly** (e.g., "kitchen" not "place1")
6. **Use emergency stop training** - practice saying "stop" commands
7. **Check map bounds** - don't send robot outside mapped area

---

**Last Updated**: January 2026
**System**: Jetson Orin Nano + ROS 2 Humble + Ollama LLM
**Status**: ✅ Production Ready
