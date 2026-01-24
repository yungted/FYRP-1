# 🚀 DEPLOYMENT CHECKLIST
## Robert Robot LLM Navigation System

**Project**: Jetson Orin Nano + ROS 2 Humble + Ollama LLM  
**Date**: January 24, 2026  
**Status**: Ready for Deployment

---

## ✅ PRE-DEPLOYMENT VERIFICATION

### Files Present
- [ ] `LLM Interface with ROS output for Pi.py` - PC controller
- [ ] `waypoint_manager.py` - Waypoint module
- [ ] `mqtt_ros_bridge_ENHANCED.py` - Jetson bridge
- [ ] `system_test.py` - Test utility
- [ ] `SETUP_AND_USAGE_GUIDE.md` - Complete guide
- [ ] `QUICK_REFERENCE.md` - Quick help
- [ ] `IMPLEMENTATION_SUMMARY.md` - Architecture
- [ ] `PC_CODE_MODIFICATIONS.md` - Change log
- [ ] `DELIVERABLES_SUMMARY.txt` - This summary
- [ ] `DEPLOYMENT_CHECKLIST.md` - This checklist

### Hardware Ready
- [ ] Jetson Orin Nano connected to power
- [ ] Jetson network (WiFi or Ethernet) connected
- [ ] PC/Laptop ready with Python 3.8+
- [ ] Robot motors and sensors working
- [ ] Lidar/sensors functional
- [ ] Microphone available for voice input

### Network Setup
- [ ] MQTT Broker IP identified (e.g., 192.168.0.165)
- [ ] MQTT Broker accessible: `ping <MQTT_IP>`
- [ ] PC can reach MQTT Broker
- [ ] Jetson can reach MQTT Broker
- [ ] Network is stable (latency < 100ms)
- [ ] WiFi signal strength adequate

---

## ✅ PC SIDE SETUP

### Python Environment
- [ ] Python 3.8+ installed: `python3 --version`
- [ ] pip package manager working: `pip3 --version`

### Dependencies Installation
- [ ] Install MQTT: `pip3 install paho-mqtt`
- [ ] Install audio: `pip3 install sounddevice numpy`
- [ ] Install OpenCV: `pip3 install opencv-python pillow`
- [ ] Install TTS: `pip3 install TTS ollama moondream`
- [ ] Install websockets: `pip3 install websockets`

### Verify Installations
```bash
python3 -c "import paho.mqtt"
python3 -c "import sounddevice"
python3 -c "import cv2"
python3 -c "import numpy"
python3 -c "from TTS.api import TTS"
```
- [ ] All imports successful

### API Keys Configuration
- [ ] Get Deepgram API key from deepgram.com
- [ ] Get Ollama API key from ollama provider
- [ ] Get Moondream API key from moondream.ai
- [ ] Edit: `LLM Interface with ROS output for Pi.py`
- [ ] Add keys to DEEPGRAM_KEY, OLLAMA_KEY, MOONDREAM_KEY
- [ ] Save file

### Configure MQTT Broker
- [ ] Edit: `LLM Interface with ROS output for Pi.py`
- [ ] Set: `MQTT_BROKER = "<your_mqtt_ip>"`
- [ ] Test: `mosquitto_sub -h <IP> -t test`
- [ ] Save file

### Verify PC Setup
```bash
cd /path/to/project
python3 system_test.py --test all
```
- [ ] All tests pass
- [ ] MQTT connectivity OK
- [ ] Network connectivity OK
- [ ] Files present and valid

---

## ✅ JETSON SIDE SETUP

### ROS 2 Installation
- [ ] ROS 2 Humble installed: `ros2 --version`
- [ ] Navigation2 installed: `ros2 pkg find nav2_bringup`
- [ ] AMCL available: `ros2 pkg find amcl`
- [ ] DiffBot base controller: `ros2 pkg find diffbot_base`

### Python Dependencies (Jetson)
```bash
sudo apt-get update
sudo apt-get install python3-paho-mqtt python3-nav2-simple-commander
```
- [ ] Installation successful

### Environment Setup (Jetson)
- [ ] ROS 2 setup sourced: `source /opt/ros/humble/setup.bash`
- [ ] Workspace sourced: `source ~/ros2_ws/install/setup.bash`
- [ ] Verify: `echo $ROS_DISTRO` shows "humble"

### Configuration Files
- [ ] Jetson map file available: `/path/to/map.yaml`
- [ ] Map file accessible to Nav2
- [ ] AMCL config file ready (if custom)
- [ ] Base controller config ready (if custom)

### Copy Jetson Bridge
```bash
scp mqtt_ros_bridge_ENHANCED.py jetson@<JETSON_IP>:~/
```
- [ ] File copied successfully

### Edit Jetson Bridge
- [ ] SSH to Jetson: `ssh jetson@<IP>`
- [ ] Edit: `mqtt_ros_bridge_ENHANCED.py`
- [ ] Set: `MQTT_BROKER = "<MQTT_IP>"`
- [ ] Verify ROS topics are correct (check with `ros2 topic list`)
- [ ] Save file

### Verify Jetson Setup
```bash
ros2 node list                          # Check ROS running
ros2 topic list | grep amcl             # Check AMCL available
ros2 action list | grep navigate        # Check Nav2 available
python3 -c "import paho.mqtt"           # Check MQTT available
```
- [ ] All checks pass

---

## ✅ SYSTEM INTEGRATION TESTS

### Test 1: MQTT Connectivity
```bash
# Terminal 1 (PC): Subscribe
mosquitto_sub -h <MQTT_IP> -t "robot/test"

# Terminal 2 (PC): Publish
mosquitto_pub -h <MQTT_IP> -t "robot/test" -m "hello"

# Check: Message received in Terminal 1
```
- [ ] Test passed

### Test 2: ROS Topics (Jetson)
```bash
ros2 topic list | grep -E "(cmd_vel|amcl_pose|odom)"
```
- [ ] Expected topics present:
  - [ ] `/cmd_vel` or `/diffbot_base_controller/cmd_vel_unstamped`
  - [ ] `/amcl_pose`
  - [ ] `/odom`

### Test 3: Nav2 Availability (Jetson)
```bash
# Start Nav2
ros2 launch nav2_bringup navigation_launch.py map:=/path/to/map.yaml

# In another terminal
ros2 action list | grep navigate
```
- [ ] `/navigate_to_pose` action available

### Test 4: Network Latency
```bash
ping <JETSON_IP>
```
- [ ] Latency < 100ms
- [ ] No packet loss

### Test 5: Full Diagnostics (PC)
```bash
python3 system_test.py --test all
```
- [ ] **All tests pass**:
  - [ ] Network connectivity
  - [ ] MQTT pub/sub
  - [ ] Waypoint manager
  - [ ] File integrity
  - [ ] ROS connectivity (if available)

---

## ✅ PRE-FLIGHT CHECKS

### Robot Physical Checks
- [ ] Motors respond to commands
- [ ] Lidar is scanning
- [ ] IMU operational
- [ ] Wheels can rotate freely
- [ ] No mechanical issues

### Robot Software Checks
```bash
# Terminal 1: Start Nav2
ros2 launch nav2_bringup navigation_launch.py map:=/path/to/map.yaml use_sim_time:=false

# Check logs for errors
# Look for: "Map loaded successfully"
```
- [ ] Nav2 starts without errors

### PC LLM Interface Check
```bash
python3 "LLM Interface with ROS output for Pi.py"
```
- [ ] Process starts
- [ ] TTS model loads: "✅ TTS Model Loaded"
- [ ] MQTT connects: "✅ MQTT connected"
- [ ] Listens for wake word: "✅ Listening... Say 'Hey Robert'!"

### Jetson Bridge Check
```bash
python3 mqtt_ros_bridge_ENHANCED.py
```
- [ ] Process starts
- [ ] MQTT connects: "📡 MQTT connected"
- [ ] AMCL pose available: "📍 Pose: ..."

### Manual Movement Test
```
Say: "Hey Robert, move forward"
Expected:
- Beep sound acknowledgment
- "Moving forward" spoken
- Robot moves forward
- Console shows: "Command completed"
```
- [ ] Test passed

### Waypoint Save Test
```
Say: "Hey Robert, save this as test_location"
Expected:
- "Saved this location as test_location" spoken
- File waypoints.json created/updated
```
- [ ] Test passed
- [ ] Check: `cat waypoints.json` shows test_location

### Waypoint Navigation Test
```
Say: "Hey Robert, go to test_location"
Expected:
- "Navigating to test_location" spoken
- Robot navigates using Nav2
- "Reached test_location" spoken (or feedback message)
```
- [ ] Test passed

### Emergency Stop Test
```
Say: "Hey Robert, emergency!"
Expected:
- Robot immediately stops
- "Emergency stop activated" spoken
- No more commands execute
Say: "Hey Robert, resume" or "clear"
Expected:
- "Emergency stop cleared" spoken
- Commands work again
```
- [ ] Test passed

---

## ✅ SAFETY VERIFICATION

### Speed Verification
```
Say: "Hey Robert, go as fast as possible"
Expected:
- Refusal: "I'm sorry, I can only move at safe speeds..."
- Robot does NOT move
```
- [ ] Safety enforced

### Distance Verification
```
Say: "Hey Robert, move forward 100 meters"
Expected:
- Clamped: Distance limited to 5m maximum
- Robot moves ~5m
```
- [ ] Safety enforced

### Duration Verification
```
Say: "Hey Robert, move for 1 hour"
Expected:
- Clamped: Duration limited to 15s maximum
- Robot moves ~15s
```
- [ ] Safety enforced

### Command Validation
All console output should show velocity clamping:
```
✅ Velocity within limits
✅ Duration within limits
✅ Command in whitelist
```
- [ ] All validations working

---

## ✅ PERFORMANCE VERIFICATION

### Response Time Test
- [ ] Voice → Beep: < 2 seconds
- [ ] Beep → TTS Response: < 5 seconds
- [ ] Total latency acceptable: < 10 seconds

### Command Execution
- [ ] Robot starts movement immediately after MQTT publish
- [ ] Movement stops at correct time
- [ ] No jerky or unstable motion

### Navigation Performance
- [ ] Waypoint navigation starts within 5 seconds
- [ ] Nav2 path planning reasonable
- [ ] Obstacle avoidance working (test with obstacle if available)

---

## ✅ FINAL DEPLOYMENT

### Backup Critical Files
- [ ] Backup robot's map: `cp map.yaml map.yaml.backup`
- [ ] Backup nav2 config: `cp nav2_params.yaml nav2_params.yaml.backup`
- [ ] Backup original bridge: `cp mqtt_ros_bridge_FIXED.py mqtt_ros_bridge_FIXED.py.backup`

### Set Appropriate Permissions
```bash
chmod +x mqtt_ros_bridge_ENHANCED.py
chmod +x system_test.py
chmod 600 <SENSITIVE_CONFIG_FILES>  # If any
```
- [ ] Permissions set

### Create Startup Scripts (Optional)
Create `~/start_robot.sh` on Jetson:
```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Terminal 1
tmux new-session -d -s nav2 "ros2 launch nav2_bringup ..."

# Terminal 2
tmux new-window -t nav2 -n bridge "python3 mqtt_ros_bridge_ENHANCED.py"

echo "Robot started in tmux session 'nav2'"
tmux attach-session -t nav2
```
- [ ] Startup script created (optional)

### Documentation Review
- [ ] Read: QUICK_REFERENCE.md
- [ ] Understand: SETUP_AND_USAGE_GUIDE.md
- [ ] Bookmark: Troubleshooting section
- [ ] Know: Emergency procedures

### Team Training
- [ ] Demonstrate basic commands
- [ ] Show waypoint saving
- [ ] Show emergency stop
- [ ] Explain safety limits
- [ ] Practice common issues

---

## ✅ DEPLOYMENT SIGN-OFF

### Technical Review
- [ ] All files present and verified
- [ ] All dependencies installed
- [ ] All tests passing
- [ ] All safety features working
- [ ] Performance acceptable
- [ ] No error messages in logs

### Documentation Review
- [ ] Setup guide read
- [ ] Quick reference reviewed
- [ ] Troubleshooting understood
- [ ] Safety procedures known

### Stakeholder Sign-Off
- [ ] Project requirements met
- [ ] System performs as expected
- [ ] Documentation adequate
- [ ] Ready for production use

---

## 📋 DEPLOYMENT STATUS

**Current Date**: _______________  
**Deployed By**: _______________  
**Verified By**: _______________  

**Overall Status**: 
- [ ] ✅ Ready for Production
- [ ] ⚠️ Minor issues (document below)
- [ ] ❌ Critical issues (resolve before deployment)

**Issues Found**:
```
[Document any issues found here]
```

**Resolution**:
```
[Document how issues were resolved]
```

---

## 🚀 LAUNCH!

Once all checkboxes are marked, you're ready to:

1. **Start MQTT Broker**
2. **Start Nav2 on Jetson**
3. **Start MQTT Bridge on Jetson**
4. **Start LLM Interface on PC**
5. **Say: "Hey Robert, move forward"**
6. **Enjoy your intelligent robot! 🤖**

---

## 📞 POST-DEPLOYMENT

### Monitoring
- [ ] Monitor first hour closely
- [ ] Watch for any errors
- [ ] Note any unusual behavior
- [ ] Collect performance data

### Optimization
- [ ] Adjust calibration if needed
- [ ] Tune response times
- [ ] Optimize waypoint locations
- [ ] Gather user feedback

### Maintenance
- [ ] Daily: Check MQTT connectivity
- [ ] Weekly: Review logs
- [ ] Monthly: Update calibration
- [ ] As needed: Troubleshoot issues

---

**Deployment Checklist Version**: 1.0  
**Last Updated**: January 24, 2026  
**Status**: ✅ Ready for Use

---

## 📚 Reference Documents

| Document | Purpose |
|----------|---------|
| QUICK_REFERENCE.md | Quick start and commands |
| SETUP_AND_USAGE_GUIDE.md | Detailed setup and troubleshooting |
| IMPLEMENTATION_SUMMARY.md | Architecture and design |
| PC_CODE_MODIFICATIONS.md | Code changes explained |
| DELIVERABLES_SUMMARY.txt | What's included |

---

**Good luck with your deployment! 🚀**
