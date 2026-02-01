#!/usr/bin/env python3
"""
PC-side MCP + LLM interface (Rewritten, robust executor).

Features:
- Listens to Deepgram for wake word + speech.
- Sends user commands to Ollama.
- Ollama returns {"reply": "...", "action": ...} where action may be an object or a list.
- Executor supports multi-step actions (control_device, delay, request_status, request_devices).
- Loads device registry from MQTT (robot/device_registry or robot/registry legacy) and saves canonical devices.json.
- Merges status updates from MQTT into device_states for LLM context.
- Validates and publishes control messages to MQTT (robot/control).
- Sends silence during TTS to keep Deepgram connection alive.
"""

import asyncio
import json
import os
import re
from datetime import datetime
import websockets
import sounddevice as sd
import numpy as np
import paho.mqtt.client as mqtt
from ollama import Client
import cv2
from PIL import Image
import moondream as md
from TTS.api import TTS
from waypoints.waypoint_manager import WaypointManager, extract_waypoint_name_from_command

# ----------------------------
# CONFIGURATION
# ----------------------------
DEEPGRAM_KEY = "dc36253a111632480faebd24920862ea1265df6a"
OLLAMA_KEY = "add508d2d17443e8b9953322d3751470.vmwVMnfKMep0zN42BPIMoidp"
MOONDREAM_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJrZXlfaWQiOiIxZWI5Njk0MS02MDVlLTQ3ZTYtYjI5NS01NjdjMGRjYWZhYTkiLCJvcmdfaWQiOiJlZzJ2bXo4a1FWY0xlOTExQVl5bkM0NU44YkhEYTMwTyIsImlhdCI6MTc2MjkzMzIzOSwidmVyIjoxfQ.onaMxtqh-X14ggd8XUDfiALxzYuHuUyI3GMbWHBYCqw"

CAM_INDEX = 2  # Default webcam index
WAKE_WORDS = ["hey robert", "hey, robert", "hi, robert", "okay, robert"]
MQTT_BROKER = "192.168.0.165"

# Topics (subscribe to both legacy and MCP topics for compatibility)
CONTROL_TOPIC = "robot/control"
STATUS_TOPIC = "robot/status"
REQUEST_TOPIC = "robot/request"
REGISTRY_TOPIC_LEGACY = "robot/registry"
REGISTRY_TOPIC_MCP = "robot/device_registry"
DEVICES_FILE = "devices.json"
PICTURES_FOLDER = "Pictures"  # Folder to store captured photos

# ROS Command Topics
ROS_CMD_TOPIC = "robot/ros_cmd"          # Publish ROS commands here
ROS_FEEDBACK_TOPIC = "robot/ros_feedback" # Receive feedback from Jetson

# Nav2 Relative Movement Topic (NEW - uses NavigateToPose with relative goals)
NAV2_RELATIVE_MOVE_TOPIC = "robot/nav2_relative_move"  # Obstacle-aware relative movements

# Navigation Topics
WAYPOINT_NAV_TOPIC = "robot/navigate_waypoint"  # Send waypoint navigation requests
WAYPOINT_FEEDBACK_TOPIC = "robot/waypoint_feedback"  # Receive navigation feedback
CURRENT_POSE_TOPIC = "robot/current_pose"  # Subscribe to robot's current pose

# ----------------------------
# SAFE ROS COMMAND CONFIGURATION
# ----------------------------
# Maximum safe velocities for service robot (m/s and rad/s)
MAX_LINEAR_VELOCITY = 0.20   # 0.20 m/s max - safe indoor speed
MAX_ANGULAR_VELOCITY = 0.4   # 0.4 rad/s max rotation
DEFAULT_MOVE_DURATION = 3.0  # Default duration for movements (seconds)

# Safety limits for modular commands
MAX_DISTANCE_METERS = 5.0    # Maximum allowed distance in one command
MAX_DURATION_SECONDS = 30.0  # Maximum allowed duration (Nav2 handles this)
MAX_ANGLE_DEGREES = 180.0    # Maximum rotation in one command

# Nav2 Navigation Speed (passed to Nav2 for path execution)
NAV2_MAX_SPEED = 0.20        # m/s - Nav2 will use this as max speed

# Default distances/angles for voice commands without specific values
DEFAULT_FORWARD_DISTANCE = 1.0   # meters - "move forward" without distance
DEFAULT_BACKWARD_DISTANCE = 0.5  # meters - "move backward" without distance  
DEFAULT_TURN_ANGLE = 90.0        # degrees - "turn left/right" without angle

# ===========================================
# MOVEMENT COMMANDS - Nav2 Relative Navigation
# ===========================================
# These commands use Nav2's NavigateToPose with relative goal calculation
# The robot will plan a path and avoid obstacles automatically!

# Command definitions with default values
# distance: meters, angle: degrees, duration: seconds (for time-based moves)
MOVEMENT_COMMANDS = {
    "move_forward": {"type": "linear", "direction": 1, "default_distance": 1.0},
    "move_backward": {"type": "linear", "direction": -1, "default_distance": 0.5},
    "turn_left": {"type": "rotation", "direction": 1, "default_angle": 90},
    "turn_right": {"type": "rotation", "direction": -1, "default_angle": 90},
    "stop": {"type": "stop"},
    "approach": {"type": "linear", "direction": 1, "default_distance": 0.3},
    "retreat": {"type": "linear", "direction": -1, "default_distance": 0.3},
    "rotate_180": {"type": "rotation", "direction": 1, "default_angle": 180},
    "slight_left": {"type": "rotation", "direction": 1, "default_angle": 30},
    "slight_right": {"type": "rotation", "direction": -1, "default_angle": 30},
}

# Emergency stop flag
emergency_stop_active = False

# ----------------------------
# GLOBAL OBJECTS
# ----------------------------
print("⏳ Loading TTS Model (Glow-TTS - optimized for speed)...")
tts_engine = TTS("tts_models/en/ljspeech/glow-tts", gpu=False)  # Set gpu=True if you have CUDA
print("✅ TTS Model Loaded.")

mqtt_client = mqtt.Client(client_id="pc-llm-client")

os.environ["OLLAMA_API_KEY"] = OLLAMA_KEY
ollama_client = Client(host="https://ollama.com", headers={"Authorization": f"Bearer {OLLAMA_KEY}"})

moondream_model = md.vl(endpoint="https://api.moondream.ai/v1", api_key=MOONDREAM_KEY)

# State
is_awake = False
command_buffer = ""
microphone_active = True  # send silence when False

device_states = {}   # flattened live telemetry
device_registry = {} # canonical registry object (MCP-like)

# Waypoint management
waypoint_manager = WaypointManager()
current_robot_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}  # Current pose from amcl_pose
robot_nav_status = "idle"  # idle, navigating, reached, failed

# ----------------------------
# Utilities: registry file handling + validation helpers
# ----------------------------
def save_registry_to_file(obj):
    try:
        with open(DEVICES_FILE, "w") as f:
            json.dump(obj, f, indent=2)
        print("💾 Saved device registry to", DEVICES_FILE)
    except Exception as e:
        print("⚠️ Failed to save registry:", e)

def parse_and_store_registry(payload_obj):
    """
    Normalize either legacy or MCP-style registry into a canonical structure:
    canonical = {"device_id":..., "timestamp":..., "devices":[{id,name,type,actions,meta}, ...]}
    """
    global device_registry
    if not isinstance(payload_obj, dict):
        print("⚠️ Registry payload is not an object")
        return

    # If legacy format with 'outputs'
    if "outputs" in payload_obj:
        canonical = {
            "device_id": payload_obj.get("device_id", payload_obj.get("id", "unknown")),
            "timestamp": payload_obj.get("timestamp"),
            "devices": []
        }
        for o in payload_obj.get("outputs", []):
            name = o.get("name") or "unknown"
            canonical["devices"].append({
                "id": (o.get("name") or name).lower(),
                "name": name,
                "type": o.get("type"),
                "actions": {a: {} for a in (o.get("actions") or [])},
                "meta": {k: v for k, v in o.items() if k not in ("name", "type", "actions")}
            })
        device_registry = canonical
        save_registry_to_file(canonical)
        return

    # If MCP style with 'devices'
    if "devices" in payload_obj:
        # ensure ids exist and are lowercase
        for d in payload_obj["devices"]:
            if "id" not in d and "name" in d:
                d["id"] = d["name"].lower()
            if "id" in d:
                d["id"] = str(d["id"]).lower()
        device_registry = payload_obj
        save_registry_to_file(payload_obj)
        return

    # Unknown structure -> store raw and write file
    device_registry = payload_obj
    save_registry_to_file(payload_obj)

def find_device_by_id(dev_id):
    """Find device dict by id or name (case-insensitive)."""
    if not device_registry:
        return None
    devices = device_registry.get("devices") or []
    if not isinstance(devices, list):
        return None
    target = str(dev_id).lower()
    for d in devices:
        if (d.get("id") and str(d.get("id")).lower() == target) or \
           (d.get("name") and str(d.get("name")).lower() == target):
            return d
    return None

# ----------------------------
# TTS (send silence during TTS)
# ----------------------------
def speak_sync(text):
    if not text:
        return
    try:
        audio = tts_engine.tts(text=text)
        audio = np.array(audio, dtype=np.float32)
        sd.play(audio, samplerate=tts_engine.synthesizer.output_sample_rate)
        sd.wait()
    except Exception as e:
        print(f"⚠️ TTS Error: {e}")

async def speak(text):
    global microphone_active
    print(f"🤖 Speaking: '{text}'")
    microphone_active = False
    await asyncio.to_thread(speak_sync, text)
    microphone_active = True

# ----------------------------
# MQTT Callbacks
# ----------------------------
def on_connect(client, userdata, flags, rc):
    print(f"✅ MQTT connected ({MQTT_BROKER}) rc={rc}")
    client.subscribe(STATUS_TOPIC)
    client.subscribe(REGISTRY_TOPIC_LEGACY)
    client.subscribe(REGISTRY_TOPIC_MCP)
    client.subscribe(ROS_FEEDBACK_TOPIC)  # Subscribe to ROS feedback from Jetson
    client.subscribe(WAYPOINT_FEEDBACK_TOPIC)  # Subscribe to waypoint navigation feedback
    client.subscribe(CURRENT_POSE_TOPIC)  # Subscribe to robot's current pose
    # Request status/devices on connect so Pi can respond (retained + active)
    client.publish(REQUEST_TOPIC, json.dumps({"request": "status"}))
    client.publish(REQUEST_TOPIC, json.dumps({"request": "devices"}))

def on_message(client, userdata, msg):
    global device_states, device_registry, emergency_stop_active, robot_nav_status, current_robot_pose
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        print("⚠️ Non-JSON MQTT payload on", msg.topic)
        return

    # Handle current pose updates (from amcl_pose subscriber on Jetson)
    if msg.topic == CURRENT_POSE_TOPIC:
        try:
            current_robot_pose = {
                "x": float(payload.get("x", 0.0)),
                "y": float(payload.get("y", 0.0)),
                "z": float(payload.get("z", 0.0)),
                "w": float(payload.get("w", 1.0))
            }
            print(f"📍 Current pose: ({current_robot_pose['x']:.2f}, {current_robot_pose['y']:.2f})")
        except Exception as e:
            print(f"⚠️ Error parsing pose: {e}")
        return

    # Handle waypoint navigation feedback
    if msg.topic == WAYPOINT_FEEDBACK_TOPIC:
        feedback_type = payload.get("type", "unknown")
        if feedback_type == "navigation_started":
            waypoint = payload.get("waypoint", "unknown")
            robot_nav_status = "navigating"
            print(f"🗺️ Started navigation to waypoint: {waypoint}")
        elif feedback_type == "waypoint_reached":
            waypoint = payload.get("waypoint", "unknown")
            robot_nav_status = "reached"
            waypoint_manager.mark_visited(waypoint)
            print(f"✅ Reached waypoint: {waypoint}")
        elif feedback_type == "navigation_failed":
            reason = payload.get("reason", "unknown")
            robot_nav_status = "failed"
            print(f"❌ Navigation failed: {reason}")
        else:
            print(f"📡 Waypoint Feedback: {payload}")
        return

    # Handle ROS feedback from Jetson
    if msg.topic == ROS_FEEDBACK_TOPIC:
        feedback_type = payload.get("type", "unknown")
        if feedback_type == "command_received":
            print(f"✅ Jetson received command: {payload.get('command')}")
        elif feedback_type == "command_completed":
            print(f"✅ Jetson completed command: {payload.get('command')}")
        elif feedback_type == "error":
            print(f"❌ Jetson error: {payload.get('message')}")
        elif feedback_type == "emergency_stop":
            emergency_stop_active = True
            print("🛑 Jetson reported emergency stop!")
        else:
            print(f"📡 ROS Feedback: {payload}")
        return

    if msg.topic in (REGISTRY_TOPIC_LEGACY, REGISTRY_TOPIC_MCP):
        print("📦 Received device registry.")
        parse_and_store_registry(payload)
        return

    if msg.topic == STATUS_TOPIC:
        # Accept MCP-style {"state": {...}} or legacy flat dicts
        if isinstance(payload, dict) and "state" in payload and isinstance(payload["state"], dict):
            device_states.update(payload["state"])
        else:
            device_states.update(payload)
        print(f"📊 Device states updated: {device_states}")
        return

# attach and start MQTT
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)
mqtt_client.loop_start()

# ----------------------------
# Publish (validate before publish)
# ----------------------------
def publish_control_message(msg: dict):
    """
    Accepts:
      - MCP parameter style: {"mcp_action":"control_device","parameters":[{...}, ...]}
      - Simple style: {"device":"led", "action":"on", ...}
    Validates device ids against device_registry before publishing.
    """
    try:
        # MCP-style action object -> publish canonical parameters to CONTROL_TOPIC
        if msg.get("mcp_action") == "control_device" and isinstance(msg.get("parameters"), list):
            valid_params = []
            for p in msg["parameters"]:
                dev = (p.get("device") or "").lower()
                if not dev:
                    print("⚠️ Parameter missing 'device', skipping:", p)
                    continue
                if not find_device_by_id(dev):
                    print(f"⚠️ Unknown device '{dev}', skipping param.")
                    continue
                valid_params.append(p)
            if not valid_params:
                print("⚠️ No valid parameters to publish.")
                return
            out = {"mcp_action": "control_device", "parameters": valid_params}
            mqtt_client.publish(CONTROL_TOPIC, json.dumps(out))
            print(f"📤 Published MCP control: {out}")
            return

        # Simple style: publish if device known
        device = (msg.get("device") or "").lower()
        if device:
            if not find_device_by_id(device):
                print(f"⚠️ Unknown device '{device}', not publishing.")
                return
            mqtt_client.publish(CONTROL_TOPIC, json.dumps(msg))
            print(f"📤 Published simple control: {msg}")
            return

        # Fallback: publish raw (least safe)
        mqtt_client.publish(CONTROL_TOPIC, json.dumps(msg))
        print(f"📤 Published raw control: {msg}")
    except Exception as e:
        print("⚠️ Publish error:", e)

# ----------------------------
# ROS Command Validation and Publishing
# ----------------------------
def validate_ros_velocity(linear_x, angular_z):
    """
    Clamp velocities to safe limits and return validated values.
    Returns (clamped_linear, clamped_angular, was_modified)
    """
    clamped_linear = max(-MAX_LINEAR_VELOCITY, min(MAX_LINEAR_VELOCITY, linear_x))
    clamped_angular = max(-MAX_ANGULAR_VELOCITY, min(MAX_ANGULAR_VELOCITY, angular_z))
    was_modified = (clamped_linear != linear_x) or (clamped_angular != angular_z)
    if was_modified:
        print(f"⚠️ Velocity clamped: linear {linear_x}->{clamped_linear}, angular {angular_z}->{clamped_angular}")
    return clamped_linear, clamped_angular, was_modified

def calculate_duration_from_distance(distance_meters, velocity):
    """Calculate estimated duration for Nav2 timeout purposes."""
    if velocity == 0:
        return 30.0  # Default timeout
    safe_distance = max(0, min(MAX_DISTANCE_METERS, abs(distance_meters)))
    # Add buffer time for acceleration/deceleration
    return min((safe_distance / abs(velocity)) * 2 + 5.0, MAX_DURATION_SECONDS)


def publish_nav2_movement(command_name, custom_params=None):
    """
    Publish a Nav2-based movement command to the Jetson via MQTT.
    
    This uses Nav2's NavigateToPose with relative goal calculation.
    The Jetson will:
    1. Get current robot pose
    2. Calculate target pose based on movement request
    3. Use Nav2 to navigate with full obstacle avoidance
    
    Supports:
    - distance (meters): "move forward 1.5 meters"
    - angle (degrees): "turn left 90 degrees"  
    - duration (seconds): "move forward for 3 seconds" -> converts to distance
    
    This is SAFE - Nav2 handles obstacle avoidance automatically!
    """
    global emergency_stop_active
    
    if emergency_stop_active and command_name != "stop":
        print("🛑 Emergency stop active - only 'stop' commands allowed")
        return False
    
    command_name = command_name.lower().strip()
    
    # Handle stop command immediately
    if command_name == "stop":
        stop_msg = {"command": "stop", "cancel_navigation": True}
        mqtt_client.publish(NAV2_RELATIVE_MOVE_TOPIC, json.dumps(stop_msg))
        print("🛑 Stop command sent - cancelling any active navigation")
        return True
    
    # Check if command is valid
    if command_name not in MOVEMENT_COMMANDS:
        print(f"⚠️ Unknown movement command '{command_name}'")
        print(f"   Available: {list(MOVEMENT_COMMANDS.keys())}")
        return False
    
    cmd_config = MOVEMENT_COMMANDS[command_name]
    cmd_type = cmd_config.get("type", "linear")
    direction = cmd_config.get("direction", 1)
    
    # Extract parameters from LLM output
    distance = None
    angle = None
    duration = None
    
    if custom_params and isinstance(custom_params, dict):
        # Distance in meters
        if "distance" in custom_params:
            distance = float(custom_params["distance"])
        elif "distance_meters" in custom_params:
            distance = float(custom_params["distance_meters"])
        
        # Angle in degrees
        if "angle" in custom_params:
            angle = float(custom_params["angle"])
        elif "angle_degrees" in custom_params:
            angle = float(custom_params["angle_degrees"])
        
        # Duration in seconds (convert to distance)
        if "duration" in custom_params:
            duration = float(custom_params["duration"])
    
    # Build the Nav2 relative movement message
    nav2_msg = {
        "command": command_name,
        "type": cmd_type,
        "direction": direction,
        "max_speed": NAV2_MAX_SPEED,
    }
    
    if cmd_type == "linear":
        # Linear movement (forward/backward)
        if distance is not None:
            # User specified distance
            nav2_msg["distance_meters"] = min(abs(distance), MAX_DISTANCE_METERS) * direction
        elif duration is not None:
            # Convert duration to distance: distance = speed * time
            estimated_distance = NAV2_MAX_SPEED * min(duration, MAX_DURATION_SECONDS)
            nav2_msg["distance_meters"] = min(estimated_distance, MAX_DISTANCE_METERS) * direction
        else:
            # Use default distance for this command
            default_dist = cmd_config.get("default_distance", DEFAULT_FORWARD_DISTANCE)
            nav2_msg["distance_meters"] = default_dist * direction
        
        # Set timeout based on distance
        timeout = calculate_duration_from_distance(abs(nav2_msg["distance_meters"]), NAV2_MAX_SPEED)
        nav2_msg["timeout_seconds"] = timeout
        
        print(f"🗺️ Nav2 Move: {nav2_msg['distance_meters']:.2f}m (timeout: {timeout:.1f}s) - obstacle-aware!")
        
    elif cmd_type == "rotation":
        # Rotation (turn left/right)
        if angle is not None:
            # User specified angle
            nav2_msg["angle_degrees"] = min(abs(angle), MAX_ANGLE_DEGREES) * direction
        elif duration is not None:
            # Convert duration to angle estimate (rough conversion)
            estimated_angle = (duration / 2.0) * 90  # ~90 degrees per 2 seconds
            nav2_msg["angle_degrees"] = min(estimated_angle, MAX_ANGLE_DEGREES) * direction
        else:
            # Use default angle for this command
            default_angle = cmd_config.get("default_angle", DEFAULT_TURN_ANGLE)
            nav2_msg["angle_degrees"] = default_angle * direction
        
        # Set timeout for rotation
        nav2_msg["timeout_seconds"] = 15.0  # Rotations should be quick
        
        print(f"🗺️ Nav2 Rotate: {nav2_msg['angle_degrees']:.1f}° (timeout: {nav2_msg['timeout_seconds']:.1f}s) - obstacle-aware!")
    
    # Publish to Nav2 relative movement topic
    mqtt_client.publish(NAV2_RELATIVE_MOVE_TOPIC, json.dumps(nav2_msg))
    return True


# Keep legacy function name for compatibility
def publish_ros_command(command_name, custom_params=None):
    """Legacy wrapper - now uses Nav2 relative movement."""
    return publish_nav2_movement(command_name, custom_params)


def emergency_stop():
    """Immediately stop the robot and cancel any navigation."""
    global emergency_stop_active
    emergency_stop_active = True
    stop_msg = {
        "command": "emergency_stop",
        "cancel_navigation": True,
        "emergency": True
    }
    mqtt_client.publish(NAV2_RELATIVE_MOVE_TOPIC, json.dumps(stop_msg))
    mqtt_client.publish(ROS_CMD_TOPIC, json.dumps(stop_msg))  # Also send to legacy topic
    print("🛑 EMERGENCY STOP PUBLISHED")


def clear_emergency_stop():
    """Clear emergency stop flag to allow new commands."""
    global emergency_stop_active
    emergency_stop_active = False
    print("✅ Emergency stop cleared - robot can receive commands")

# ----------------------------
# Vision Logic (unchanged)
# ----------------------------
def capture_and_analyze(prompt="Describe this image in detail."):
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return "I couldn't access the camera."
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return "I failed to capture an image."
    filename = "capture.jpg"
    cv2.imwrite(filename, frame)
    print(f"📸 Image captured: {filename}")
    try:
        image = Image.open(filename)
        print(f"🔍 Analyzing with Moondream: {prompt}")
        response = moondream_model.query(image, prompt)
        description = response.get("answer", "I couldn't see anything clearly.")
        print(f"✅ Vision: {description}")
        return description
    except Exception as e:
        print(f"⚠️ Moondream error: {e}")
        return "I had trouble analyzing the image."


def count_people_in_frame():
    """
    Capture a frame and count how many people are visible.
    Returns (count, frame, image_pil) where count is the number of people detected.
    """
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return -1, None, None
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return -1, None, None
    
    try:
        # Convert to PIL for Moondream
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_pil = Image.fromarray(frame_rgb)
        
        # Ask Moondream to count people
        response = moondream_model.query(image_pil, "How many people are in this image? Reply with just a number. If no people, say 0.")
        answer = response.get("answer", "0").strip()
        
        # Extract number from response
        import re
        numbers = re.findall(r'\d+', answer)
        count = int(numbers[0]) if numbers else 0
        print(f"👥 People detected: {count}")
        return count, frame, image_pil
    except Exception as e:
        print(f"⚠️ Error counting people: {e}")
        return 0, frame, None


def save_photo_to_pictures(frame, photo_type="photo", description=""):
    """
    Save a captured frame to the Pictures folder with timestamp.
    Returns the filepath if successful, None otherwise.
    """
    # Ensure Pictures folder exists
    os.makedirs(PICTURES_FOLDER, exist_ok=True)
    
    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_type = photo_type.replace(" ", "_").lower()
    filename = f"{safe_type}_{timestamp}.jpg"
    filepath = os.path.join(PICTURES_FOLDER, filename)
    
    try:
        cv2.imwrite(filepath, frame)
        print(f"📸 Photo saved: {filepath}")
        return filepath
    except Exception as e:
        print(f"⚠️ Error saving photo: {e}")
        return None


def analyze_scene_for_photo(image_pil, photo_type):
    """
    Analyze if the scene is good for the requested photo type.
    Returns (is_good, suggestion) tuple.
    """
    try:
        if "selfie" in photo_type.lower():
            response = moondream_model.query(image_pil, "Is there a person close to the camera suitable for a selfie? Answer yes or no, and briefly describe positioning.")
        elif "group" in photo_type.lower():
            response = moondream_model.query(image_pil, "Are the people in frame well positioned for a group photo? Are they all visible and well framed? Answer briefly.")
        else:
            response = moondream_model.query(image_pil, "Is this a good composition for a photo? Describe briefly.")
        
        answer = response.get("answer", "")
        print(f"📷 Scene analysis: {answer}")
        return answer
    except Exception as e:
        print(f"⚠️ Scene analysis error: {e}")
        return "Unable to analyze scene."

# ----------------------------
# LLM Prompt helpers
# ----------------------------
def load_devices_prompt():
    """
    Build human-readable device list for LLM from device_registry file or in-memory registry.
    """
    try:
        obj = device_registry or {}
        if not obj and os.path.exists(DEVICES_FILE):
            with open(DEVICES_FILE, "r") as f:
                obj = json.load(f)
    except Exception:
        obj = {}

    lines = []
    devices = obj.get("devices") or obj.get("outputs") or []
    for d in devices:
        name = d.get("name") or d.get("id") or "unknown"
        dtype = d.get("type", "unknown")
        actions = d.get("actions")
        if isinstance(actions, dict):
            actions_list = ", ".join(actions.keys())
        elif isinstance(actions, list):
            actions_list = ", ".join(actions)
        else:
            actions_list = str(actions)
        lines.append(f"- {name} (type:{dtype}, actions:{actions_list})")
    if not lines:
        return "(No devices found)"
    return "\n".join(lines)

def get_device_status_prompt():
    if not device_states:
        return "(No device states available)"
    lines = []
    for device, state in device_states.items():
        lines.append(f"- {device}: {state}")
    return "\n".join(lines)

# ----------------------------
# Ask Ollama (keeps device list + states in system prompt)
# ----------------------------
def get_ros_commands_prompt():
    """Return available movement commands for the LLM with descriptions."""
    command_descriptions = {
        "move_forward": "Move forward (default 1m, or specify distance in meters or duration in seconds)",
        "move_backward": "Move backward (default 0.5m, or specify distance/duration)",
        "turn_left": "Rotate left/counter-clockwise (default 90°, or specify angle in degrees)",
        "turn_right": "Rotate right/clockwise (default 90°, or specify angle in degrees)",
        "stop": "Immediately stop all movement and cancel navigation",
        "rotate_180": "Turn around (rotate 180 degrees)",
        "approach": "Slowly approach forward (default 0.3m) - for getting closer to objects/people",
        "retreat": "Slowly back away (default 0.3m) - for moving away safely",
        "slight_left": "Small left turn (30 degrees)",
        "slight_right": "Small right turn (30 degrees)",
    }
    lines = []
    for cmd, desc in command_descriptions.items():
        lines.append(f"  - {cmd}: {desc}")
    return "\n".join(lines)

def get_navigation_context_prompt():
    """
    Build navigation context for LLM including available waypoints,
    current robot pose, and navigation capabilities.
    """
    waypoint_context = waypoint_manager.get_context_for_llm()
    
    pose_info = f"Current Position: ({current_robot_pose['x']:.2f}, {current_robot_pose['y']:.2f})"
    nav_status = f"Navigation Status: {robot_nav_status}"
    
    return f"\n{pose_info}\n{nav_status}\n\n{waypoint_context}"

def ask_ollama(user_text):
    device_list = load_devices_prompt()
    device_status = get_device_status_prompt()
    ros_commands = get_ros_commands_prompt()
    nav_context = get_navigation_context_prompt()
    
    system_prompt = (
    "You are Robert, a helpful intelligent service robot assistant.\n"
    "You have access to IoT devices, a camera, and ROBOT MOVEMENT controls.\n"
    "You can navigate to saved locations (waypoints) using the navigate_to_waypoint action.\n\n"
    "DEVICE LIST (from devices.json):\n"
    f"{device_list}\n\n"
    "CURRENT DEVICE STATES (live telemetry):\n"
    f"{device_status}\n\n"
    "NAVIGATION CONTEXT:\n"
    f"{nav_context}\n\n"
    "AVAILABLE ROS MOVEMENT COMMANDS (ONLY use these exact names):\n"
    f"{ros_commands}\n\n"
    "=== RESPONSE FORMAT ===\n"
    "You MUST respond with valid JSON only. Format:\n"
    "{\"reply\": \"<verbal response>\", \"action\": <action or list of actions>}\n\n"
    "Each action must be: {\"mcp_action\": \"<type>\", \"parameters\": [<params>]}\n\n"
    "=== CONTEXT-AWARE COMMAND INTERPRETATION ===\n"
    "You are an intelligent assistant that interprets high-level user commands and converts them to low-level ROS actions.\n\n"
    "COMMAND INTERPRETATION STRATEGY:\n"
    "1. UNDERSTAND INTENT: Analyze what the user wants to achieve, not just literal commands\n"
    "2. CHECK CONTEXT: Consider robot's current pose, navigation status, and available waypoints\n"
    "3. CHOOSE METHOD: Select between:\n"
    "   a) Low-level ROS commands (move_forward, turn_left, etc.) for local navigation\n"
    "   b) Waypoint navigation (navigate_to_waypoint) for predefined locations\n"
    "4. RESPOND NATURALLY: Explain what you're doing in conversational language\n\n"
    "=== NAVIGATING TO WAYPOINTS ===\n"
    "When user mentions a location that matches a saved waypoint, use navigate_to_waypoint:\n"
    "{\"mcp_action\":\"navigate_to_waypoint\",\"parameters\":[{\"waypoint\":\"waypoint_name\"}]}\n\n"
    "Examples:\n"
    "- User: 'Go to kitchen' → Find 'kitchen' waypoint and navigate\n"
    "- User: 'Take me to the living room' → Find 'living_room' waypoint and navigate\n"
    "- User: 'Head to bedroom' → Find 'bedroom' waypoint and navigate\n\n"
    "SAVE/UPDATE WAYPOINTS:\n"
    "When user says 'save this location as <name>' or 'remember this as <name>':\n"
    "{\"mcp_action\":\"save_waypoint\",\"parameters\":[{\"waypoint_name\":\"<name>\",\"description\":\"<location description>\"}]}\n\n"
    "CLEAR/DELETE WAYPOINTS:\n"
    "When user explicitly says 'clear all waypoints', 'delete all waypoints', 'forget all locations', or 'reset waypoints':\n"
    "{\"mcp_action\":\"clear_waypoints\",\"parameters\":[]}\n"
    "⚠️ SAFETY: Only use this action if the user EXPLICITLY requests it. Never hallucinate this command.\n\n"
    "=== SAFETY-FIRST REASONING ===\n"
    "Before executing ANY movement command, you MUST evaluate:\n"
    "1. INTENT ANALYSIS: What does the user actually want to achieve?\n"
    "2. SAFETY CHECK: Is this request safe in the current context?\n"
    "3. APPROPRIATE RESPONSE: Choose the safest command that achieves the goal.\n\n"
    "SAFETY RULES (MANDATORY):\n"
    "- ONLY use commands from AVAILABLE ROS MOVEMENT COMMANDS above.\n"
    "- NEVER invent or hallucinate new movement commands or waypoints that don't exist.\n"
    "- NEVER specify velocity values - they are predefined for safety.\n"
    "- If user asks for something dangerous (e.g., 'go fast', 'maximum speed'), politely REFUSE.\n"
    "- If unsure about intent, ASK for clarification instead of guessing.\n"
    "- Use 'stop' IMMEDIATELY if user says: stop, halt, wait, danger, emergency, or indicates concern.\n"
    "- For large movements, confirm with user first or break into smaller steps.\n"
    "- When guiding people, use slow, predictable movements (slow_forward, guide_forward).\n"
    "- If a waypoint doesn't exist, suggest checking available waypoints or saving a new one.\n\n"
    "INTENT INTERPRETATION EXAMPLES (Local Movement):\n"
    "- 'Come here' / 'Come to me' → approach (slow, careful approach)\n"
    "- 'Go away' / 'Back off' → retreat (back away from user)\n"
    "- 'Follow me' / 'Come with me' → guide_forward (steady paced following)\n"
    "- 'Turn around' / 'Turn around and face me' → rotate_180 + potentially turn_left/turn_right\n"
    "- 'Face me' / 'Look at me' → turn_left or turn_right (choose based on robot's orientation)\n"
    "- 'Go left' / 'Go right' → slight_left or slight_right (forward + turn)\n"
    "- 'Just turn' → turn_left or turn_right (rotation only)\n"
    "- 'Move a bit' / 'Nudge forward' → slow_forward with small distance\n"
    "- 'STOP!' / 'Freeze!' / 'Halt!' → stop (immediate)\n\n"
    "INTENT INTERPRETATION EXAMPLES (Navigation):\n"
    "- 'Go to [location]' / 'Navigate to [location]' / 'Take me to [location]' → navigate_to_waypoint\n"
    "- 'Go to kitchen' → navigate_to_waypoint with waypoint='kitchen'\n"
    "- 'Remember this as my office' / 'Save this as office' → save_waypoint\n"
    "- 'What locations do I have saved?' / 'List waypoints' → Describe available waypoints from context\n\n"
    "MULTI-STEP SEQUENCES:\n"
    "For complex navigation, return multiple actions in order:\n"
    "{\"action\": [{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"turn_left\"}]},\n"
    "             {\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"move_forward\"}]}]}\n\n"
    "Example: User says 'Turn around and go to the kitchen'\n"
    "{\"action\": [{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"rotate_180\"}]},\n"
    "             {\"mcp_action\":\"navigate_to_waypoint\",\"parameters\":[{\"waypoint\":\"kitchen\"}]}]}\n\n"
    "=== ROS COMMAND FORMAT (MODULAR) ===\n"
    "Basic command: {\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"<command_name>\"}]}\n\n"
    "MODULAR PARAMETERS (specify distance, angle, OR duration):\n"
    "- distance (meters): For linear movements - 'move forward 2 meters'\n"
    "  {\"command\":\"move_forward\", \"distance\": 2.0}\n"
    "- angle (degrees): For rotations - 'turn left 90 degrees'\n"
    "  {\"command\":\"turn_left\", \"angle\": 90}\n"
    "- duration (seconds): Direct time control - 'move for 5 seconds'\n"
    "  {\"command\":\"move_forward\", \"duration\": 5.0}\n\n"
    "SAFETY LIMITS: Max distance=5m, Max angle=360°, Max duration=15s\n"
    "If user exceeds limits, values are automatically clamped for safety.\n\n"
    "=== VISION/CAMERA ===\n"
    "For vision requests (analyzing what's in front), use 'capture_image' with a contextual vision_prompt:\n"
    "{\"mcp_action\":\"capture_image\",\"parameters\":[{\"vision_prompt\":\"<question>\"}]}\n\n"
    "=== PHOTO CAPTURE (SELFIES, GROUP PHOTOS) ===\n"
    "For taking photos that are SAVED to the Pictures folder, use 'take_photo':\n"
    "{\"mcp_action\":\"take_photo\",\"parameters\":[{\"photo_type\":\"<type>\",\"expected_people\":<number>,\"countdown\":<seconds>}]}\n\n"
    "Photo types: 'selfie', 'group_photo', 'portrait', 'photo'\n"
    "- expected_people: Number of people that should be in the photo (optional)\n"
    "- countdown: Seconds to count down before capture (default: 3)\n\n"
    "AWARENESS FEATURE: Before capturing, I will analyze the scene to ensure:\n"
    "- For selfies: At least 1 person is visible and well-positioned\n"
    "- For group photos: The expected number of people are visible\n"
    "- Good composition and framing\n"
    "If conditions aren't met, I'll ask the user to adjust before taking the photo.\n\n"
    "Examples:\n"
    "- 'Take a selfie' → {\"mcp_action\":\"take_photo\",\"parameters\":[{\"photo_type\":\"selfie\",\"expected_people\":1}]}\n"
    "- 'Take a group photo of 4 people' → {\"mcp_action\":\"take_photo\",\"parameters\":[{\"photo_type\":\"group_photo\",\"expected_people\":4}]}\n"
    "- 'Take a picture' → {\"mcp_action\":\"take_photo\",\"parameters\":[{\"photo_type\":\"photo\"}]}\n\n"
    "=== IOT DEVICES ===\n"
    "For buzzer: use 'frequency': <Hz>, 'duration': <seconds>\n"
    "For LED/servo: use 'state': 'on'/'off' or position values\n"
    "Check device_states to avoid redundant actions (e.g., don't turn on LED if already on).\n\n"
    "=== EXAMPLES ===\n"
    "User: 'Move forward' → {\"reply\":\"Moving forward.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"move_forward\"}]}]}\n"
    "User: 'Move forward 2 meters' → {\"reply\":\"Moving forward 2 meters.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"move_forward\",\"distance\":2.0}]}]}\n"
    "User: 'Go to kitchen' → {\"reply\":\"Navigating to the kitchen now.\",\"action\":[{\"mcp_action\":\"navigate_to_waypoint\",\"parameters\":[{\"waypoint\":\"kitchen\"}]}]}\n"
    "User: 'Save this location as office' → {\"reply\":\"Saving this location as office.\",\"action\":[{\"mcp_action\":\"save_waypoint\",\"parameters\":[{\"waypoint_name\":\"office\",\"description\":\"Office location\"}]}]}\n"
    "User: 'Turn left 90 degrees' → {\"reply\":\"Turning left 90 degrees.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"turn_left\",\"angle\":90}]}]}\n"
    "User: 'Turn around and go to living room' → {\"reply\":\"Turning around and heading to the living room.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"rotate_180\"}]},{\"mcp_action\":\"navigate_to_waypoint\",\"parameters\":[{\"waypoint\":\"living_room\"}]}]}\n"
    "User: 'STOP!' → {\"reply\":\"Stopping immediately.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"stop\"}]}]}\n"
    )
    try:
        res = ollama_client.chat(
            model="gpt-oss:120b-cloud",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_text}
            ],
            stream=False
        )
        raw = res["message"]["content"]
        print(f"🤖 LLM Raw: {raw}")
        # Try to extract JSON object/array from LLM output
        match = re.search(r'(\{[\s\S]*\}|\[[\s\S]*\])', raw)
        if match:
            try:
                return json.loads(match.group(1))
            except Exception as e:
                print("⚠️ Failed to parse extracted JSON from LLM:", e)
        # fallback: if content itself looks like JSON, try to load it directly
        try:
            return json.loads(raw)
        except Exception:
            pass
    except Exception as e:
        print(f"⚠️ LLM Error: {e}")
    return None

# ----------------------------
# Action handlers (robust and extendable)
# ----------------------------
async def handle_control_device(parameters):
    """
    parameters: list of parameter dicts, each with at least 'device' and associated action/state keys.
    Validates each param and publishes using publish_control_message().
    Converts LLM-generated note formats to Pi-compatible tone_sequence format.
    """
    if not isinstance(parameters, list):
        print("⚠️ control_device parameters should be a list.")
        return

    for p in parameters:
        # Basic validation
        dev = p.get("device") or p.get("id")
        if not dev:
            print("⚠️ control parameter missing 'device':", p)
            continue

        # normalize device id
        p["device"] = str(dev).lower()

        # Validate against registry
        if not find_device_by_id(p["device"]):
            print(f"⚠️ Unknown device '{p['device']}' in control params, skipping.")
            continue

        # Convert LLM "notes" format to Pi-compatible tone_sequence
        if "notes" in p and isinstance(p["notes"], list):
            tone_sequence = []
            for note in p["notes"]:
                if isinstance(note, dict):
                    tone_sequence.append({
                        "frequency_hz": note.get("freq", note.get("frequency", 2000)),
                        "duration_s": note.get("duration", 0.5)
                    })
            if tone_sequence:
                p["tone_sequence"] = tone_sequence
            # Remove legacy keys to avoid conflicts
            del p["notes"]
            if "state" in p:
                del p["state"]

    # Publish canonical MCP style (the Pi accepts this format)
    publish_control_message({"mcp_action": "control_device", "parameters": parameters})


async def handle_delay(parameters):
    """
    parameters: list of dicts with 'duration_seconds' or single dict.
    """
    durations = []
    if isinstance(parameters, list):
        for p in parameters:
            try:
                d = float(p.get("duration_seconds", p.get("duration", 0)))
                durations.append(max(0, d))
            except Exception:
                print("⚠️ Invalid delay parameter, skipping:", p)
    elif isinstance(parameters, dict):
        try:
            d = float(parameters.get("duration_seconds", parameters.get("duration", 0)))
            durations.append(max(0, d))
        except Exception:
            pass
    # Execute delays sequentially
    for d in durations:
        print(f"⏳ Delaying for {d} seconds...")
        await asyncio.sleep(d)

async def handle_capture_image(parameters):
    """Handle vision/capture_image requests with LLM-generated prompts."""
    # Extract vision_prompt from parameters (LLM-generated)
    prompt = None
    if isinstance(parameters, list):
        for p in parameters:
            if isinstance(p, dict) and p.get("vision_prompt"):
                prompt = p.get("vision_prompt")
                break
    elif isinstance(parameters, dict):
        prompt = parameters.get("vision_prompt")
    
    # Fallback if no prompt provided
    if not prompt:
        prompt = "Describe what you see in detail."
    
    print(f"📸 Using vision prompt: {prompt}")
    description = await asyncio.to_thread(capture_and_analyze, prompt)
    await speak(f"I see {description}")


async def handle_take_photo(parameters):
    """
    Handle photo capture requests (selfie, group photo, general photo).
    
    This function is "aware" - it analyzes the scene before capturing to ensure
    the photo request can be fulfilled (e.g., checking for correct number of people).
    
    parameters: list of dicts with:
        - photo_type: "selfie", "group_photo", "photo", etc.
        - expected_people: (optional) number of people expected in the photo
        - description: (optional) description for the photo
        - countdown: (optional) countdown before taking photo (default: 3)
    """
    if not isinstance(parameters, list):
        parameters = [parameters] if isinstance(parameters, dict) else [{}]
    
    for p in parameters:
        if not isinstance(p, dict):
            p = {}
        
        photo_type = p.get("photo_type", "photo").lower()
        expected_people = p.get("expected_people", None)
        description = p.get("description", "")
        countdown = p.get("countdown", 3)
        
        # Determine expected people based on photo type if not specified
        if expected_people is None:
            if "selfie" in photo_type:
                expected_people = 1
            elif "group" in photo_type:
                expected_people = 2  # At least 2 for group
        
        print(f"📷 Photo request: {photo_type}, expecting {expected_people} people")
        
        # Step 1: Analyze the scene first
        await speak(f"Let me check if everyone is in frame for the {photo_type}.")
        
        people_count, frame, image_pil = await asyncio.to_thread(count_people_in_frame)
        
        if people_count == -1 or frame is None:
            await speak("I'm having trouble accessing the camera. Please try again.")
            continue
        
        # Step 2: Check if we have the expected number of people
        if expected_people is not None:
            if "selfie" in photo_type and people_count == 0:
                await speak("I don't see anyone in front of the camera. Please position yourself for the selfie.")
                continue
            elif "group" in photo_type:
                if people_count < expected_people:
                    await speak(f"I can only see {people_count} {'person' if people_count == 1 else 'people'}, but you asked for a photo with {expected_people}. Please make sure everyone is in frame.")
                    continue
                elif people_count > expected_people:
                    await speak(f"I see {people_count} people, which is more than the {expected_people} you mentioned. I'll take the photo with everyone visible.")
            elif people_count == 0 and photo_type in ["selfie", "portrait"]:
                await speak("I don't see anyone in the frame. Please position yourself for the photo.")
                continue
        
        # Step 3: Analyze scene composition
        if image_pil:
            scene_feedback = await asyncio.to_thread(analyze_scene_for_photo, image_pil, photo_type)
        
        # Step 4: Countdown and capture
        if countdown and countdown > 0:
            await speak(f"Perfect! I see {people_count} {'person' if people_count == 1 else 'people'}. Get ready!")
            for i in range(int(countdown), 0, -1):
                await speak(str(i))
                await asyncio.sleep(0.8)
        
        # Capture final photo
        cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
        if not cap.isOpened():
            await speak("I couldn't access the camera for the final capture.")
            continue
        ret, final_frame = cap.read()
        cap.release()
        
        if not ret:
            await speak("I failed to capture the photo. Please try again.")
            continue
        
        # Save the photo
        filepath = await asyncio.to_thread(save_photo_to_pictures, final_frame, photo_type, description)
        
        if filepath:
            # Confirm with buzzer
            publish_control_message({"device": "buzzer", "action": "beep"})
            
            if "selfie" in photo_type:
                await speak(f"Selfie captured and saved! You look great!")
            elif "group" in photo_type:
                await speak(f"Group photo captured with {people_count} people! The photo has been saved.")
            else:
                await speak(f"Photo captured and saved to the Pictures folder.")
        else:
            await speak("I captured the photo but had trouble saving it.")

async def handle_request(parameters):
    """
    Handles requests generated by the LLM such as 'status' or 'devices'.
    Expects parameters or single dict with {"request": "status"} or "devices"
    """
    if isinstance(parameters, list):
        for p in parameters:
            req = p.get("request")
            if req == "status":
                mqtt_client.publish(REQUEST_TOPIC, json.dumps({"request": "status"}))
                print("📨 Requested status from devices.")
            elif req == "devices":
                mqtt_client.publish(REQUEST_TOPIC, json.dumps({"request": "devices"}))
                print("📨 Requested devices from devices.")
    elif isinstance(parameters, dict):
        req = parameters.get("request")
        if req == "status":
            mqtt_client.publish(REQUEST_TOPIC, json.dumps({"request": "status"}))
            print("📨 Requested status from devices.")
        elif req == "devices":
            mqtt_client.publish(REQUEST_TOPIC, json.dumps({"request": "devices"}))
            print("📨 Requested devices from devices.")


async def handle_ros_command(parameters):
    """
    Handle ROS movement commands from the LLM.
    Validates commands against the whitelist and publishes to MQTT for Jetson.
    
    parameters: list of dicts, each with 'command' and optional 'duration'
    
    Safety Features:
    - Only whitelisted commands are allowed
    - Velocities are predefined and cannot be overridden by LLM
    - Duration is clamped to safe limits (0-10 seconds)
    - Emergency stop takes priority
    """
    if not isinstance(parameters, list):
        parameters = [parameters] if isinstance(parameters, dict) else []
    
    for p in parameters:
        if not isinstance(p, dict):
            print(f"⚠️ Invalid ROS command parameter: {p}")
            continue
        
        command = p.get("command", "").lower().strip()
        if not command:
            print("⚠️ ROS command parameter missing 'command' field")
            continue
        
        # Handle emergency stop commands immediately
        if command in ("emergency_stop", "e_stop", "estop"):
            emergency_stop()
            await speak("Emergency stop activated. Robot is halted.")
            return
        
        # Handle clear emergency stop
        if command in ("clear_emergency", "reset", "resume"):
            clear_emergency_stop()
            await speak("Emergency stop cleared. Robot ready for commands.")
            return
        
        # Extract modular parameters (duration, distance, angle)
        custom_params = {}
        if "duration" in p:
            custom_params["duration"] = p["duration"]
        if "distance" in p:
            custom_params["distance"] = p["distance"]
        if "distance_meters" in p:
            custom_params["distance_meters"] = p["distance_meters"]
        if "angle" in p:
            custom_params["angle"] = p["angle"]
        if "angle_degrees" in p:
            custom_params["angle_degrees"] = p["angle_degrees"]
        
        # Publish the validated command with modular parameters
        success = publish_ros_command(command, custom_params)
        
        if not success:
            # Command was rejected - inform user
            available = ", ".join(MOVEMENT_COMMANDS.keys())
            await speak(f"I can't do that movement. Available commands are: {available}")
        else:
            # Wait briefly for command to be received before next command
            await asyncio.sleep(0.1)


async def handle_navigate_to_waypoint(parameters):
    """
    Handle navigation to a saved waypoint using Nav2.
    
    parameters: list of dicts with 'waypoint' field (waypoint name)
    
    This sends a high-level navigation request to the Jetson which uses Nav2's
    navigate_to_pose action to autonomously navigate to the target.
    
    Features:
    - Waypoint validation before sending
    - Clear feedback to user on navigation start/completion
    - Error handling for non-existent waypoints
    """
    if not isinstance(parameters, list):
        parameters = [parameters] if isinstance(parameters, dict) else []
    
    for p in parameters:
        if not isinstance(p, dict):
            print(f"⚠️ Invalid navigate parameter: {p}")
            await speak("I encountered an error with the navigation parameters.")
            continue
        
        waypoint_name = p.get("waypoint", "").lower().strip()
        if not waypoint_name:
            print("⚠️ navigate_to_waypoint missing 'waypoint' field")
            await speak("I need to know which waypoint you want me to navigate to.")
            continue
        
        # Validate waypoint exists
        waypoint_data = waypoint_manager.get_waypoint(waypoint_name)
        if not waypoint_data:
            available = ", ".join(waypoint_manager.list_waypoints())
            if available:
                await speak(f"I don't have a waypoint named '{waypoint_name}'. "
                           f"Available waypoints are: {available}. "
                           f"Would you like me to save this location as '{waypoint_name}'?")
            else:
                await speak(f"I don't have any waypoints saved yet. "
                           f"You can say 'save this location as {waypoint_name}' to create one.")
            print(f"⚠️ Waypoint '{waypoint_name}' not found in registry")
            continue
        
        # Extract ROS pose from waypoint
        x, y, z, w = waypoint_data["x"], waypoint_data["y"], waypoint_data["z"], waypoint_data["w"]
        description = waypoint_data.get("description", waypoint_name)
        
        # Create navigation message for Jetson
        nav_msg = {
            "action": "navigate_to_pose",
            "waypoint": waypoint_name,
            "pose": {
                "x": x,
                "y": y,
                "z": z,
                "w": w
            },
            "description": description
        }
        
        # Publish to Jetson
        mqtt_client.publish(WAYPOINT_NAV_TOPIC, json.dumps(nav_msg))
        
        print(f"🗺️ Requesting navigation to waypoint '{waypoint_name}' at ({x:.2f}, {y:.2f})")
        await speak(f"I'm navigating to the {description}. This may take a moment.")


async def handle_save_waypoint(parameters):
    """
    Handle saving the robot's current position as a named waypoint.
    
    parameters: list of dicts with 'waypoint_name' and optional 'description'
    
    This uses the robot's current AMCL pose to save the location with a semantic name.
    The user can later reference this waypoint by name for navigation.
    """
    if not isinstance(parameters, list):
        parameters = [parameters] if isinstance(parameters, dict) else []
    
    for p in parameters:
        if not isinstance(p, dict):
            print(f"⚠️ Invalid save_waypoint parameter: {p}")
            await speak("I encountered an error saving the waypoint.")
            continue
        
        waypoint_name = p.get("waypoint_name", "").lower().strip()
        description = p.get("description", "")
        
        if not waypoint_name:
            print("⚠️ save_waypoint missing 'waypoint_name' field")
            await speak("I need a name to save this location.")
            continue
        
        # Use current robot pose
        x = current_robot_pose.get("x", 0.0)
        y = current_robot_pose.get("y", 0.0)
        z = current_robot_pose.get("z", 0.0)
        w = current_robot_pose.get("w", 1.0)
        
        # Generate description if not provided
        if not description:
            description = f"Location at ({x:.2f}, {y:.2f})"
        
        # Add or update waypoint
        success = waypoint_manager.add_waypoint(
            waypoint_name, x, y, z, w, description
        )
        
        if success:
            await speak(f"Saved this location as {waypoint_name}. I can navigate here later if you ask.")
            print(f"✅ Waypoint '{waypoint_name}' saved at ({x:.2f}, {y:.2f})")
        else:
            await speak(f"I couldn't save the waypoint '{waypoint_name}'. Please try a different name.")
            print(f"❌ Failed to save waypoint '{waypoint_name}'")


async def handle_list_waypoints(parameters):
    """
    List all available waypoints to the user.
    
    This gives the user a verbal readout of all saved waypoints and their descriptions.
    """
    waypoints = waypoint_manager.list_waypoints()
    
    if not waypoints:
        await speak("I haven't saved any waypoints yet. You can say 'save this location as' followed by a name.")
        print("📍 No waypoints available")
        return
    
    # Build a nice verbal list
    waypoint_list = []
    for name in waypoints:
        wp = waypoint_manager.get_waypoint(name)
        desc = wp.get("description", name)
        waypoint_list.append(f"{name}: {desc}")
    
    message = "Here are the locations I have saved: " + ", ".join(waypoint_list)
    await speak(message)
    print(f"📍 Listed {len(waypoints)} waypoints")


async def handle_clear_waypoints(parameters):
    """
    Handle request to delete all saved waypoints.
    
    This is a safety-critical operation that requires explicit confirmation.
    The user must say "clear all waypoints" or "delete all waypoints" to trigger.
    
    parameters: list of dicts with optional 'confirm' field (for testing)
    
    Safety Features:
    - Prints debug inventory before deletion
    - Requires explicit user confirmation
    - Prints confirmation after deletion
    - Prevents accidental data loss
    """
    # Check if there are waypoints to delete
    count = waypoint_manager.get_waypoint_count()
    
    if count == 0:
        await speak("There are no waypoints to delete.")
        print("📍 No waypoints to clear")
        return
    
    # Print current inventory for debugging
    print(f"🔍 WAYPOINT DELETION REQUESTED - Current inventory:")
    waypoint_manager.print_all_waypoints()
    
    # Perform the deletion with confirmation
    success = waypoint_manager.clear_all_waypoints(confirm=True)
    
    if success:
        await speak(f"I have deleted all {count} saved locations. You can save new ones whenever you want.")
        print(f"✅ Successfully cleared {count} waypoints")
    else:
        await speak("I couldn't delete the waypoints. Please try again.")
        print("❌ Failed to clear waypoints")


async def handle_action(action_obj):
    """
    Dispatch actions to appropriate handlers based on mcp_action type.
    
    Supported action types:
    - control_device: Control IoT devices (LED, buzzer, servo, etc.)
    - delay: Pause execution for specified duration
    - capture_image: Take image and analyze with vision model
    - request: Request status or device list from robot
    - ros_command: Execute low-level ROS movement commands
    - navigate_to_waypoint: Navigate to a saved waypoint using Nav2
    - save_waypoint: Save current robot position as a waypoint
    - list_waypoints: List all available waypoints
    - clear_waypoints: Delete all saved waypoints (safety-critical)
    
    action_obj: {"mcp_action": "...", "parameters": [...]}
    """
    if not isinstance(action_obj, dict):
        print("⚠️ action is not an object:", action_obj)
        return
    action_type = action_obj.get("mcp_action")
    params = action_obj.get("parameters", [])
    if action_type is None:
        print("⚠️ action missing 'mcp_action', skipping:", action_obj)
        return

    action_type = str(action_type).lower()

    if action_type == "control_device":
        await handle_control_device(params)
    elif action_type == "delay":
        await handle_delay(params)
    elif action_type in ("capture_image", "vision", "analyze_image"):
        await handle_capture_image(params)
    elif action_type in ("request", "ask"):
        await handle_request(params)
    elif action_type in ("ros_command", "move", "navigate", "robot_move", "movement"):
        await handle_ros_command(params)
    elif action_type in ("navigate_to_waypoint", "goto_waypoint", "navigate"):
        await handle_navigate_to_waypoint(params)
    elif action_type in ("save_waypoint", "save_location", "remember_location"):
        await handle_save_waypoint(params)
    elif action_type in ("list_waypoints", "show_waypoints", "what_waypoints"):
        await handle_list_waypoints(params)
    elif action_type in ("clear_waypoints", "delete_waypoints", "clear_all_waypoints", "delete_all_waypoints"):
        await handle_clear_waypoints(params)
    elif action_type in ("take_photo", "capture_photo", "take_picture", "selfie", "group_photo"):
        await handle_take_photo(params)
    else:
        print(f"⚠️ Unknown action type '{action_type}', skipping. Action: {action_obj}")

# ----------------------------
# Executor: Accept both single action object or list of actions
# ----------------------------
async def execute_logic(result):
    """
    result expected to be a dict like:
      {"reply": "...", "action": {..} } or {"reply":"...","action":[...]}
    """
    if not result:
        return

    # Speak (if provided)
    reply = result.get("reply") or result.get("response") or result.get("text")
    if reply:
        await speak(reply)

    action = result.get("action") or result.get("actions") or result.get("action_list")
    if not action:
        return

    # Normalize action into a list
    actions_list = []
    if isinstance(action, dict):
        actions_list = [action]
    elif isinstance(action, list):
        actions_list = action
    else:
        print("⚠️ Unexpected 'action' format from LLM:", action)
        return

    # Execute actions sequentially, but be resilient to errors
    for idx, a in enumerate(actions_list):
        try:
            print(f"➡️ Executing action {idx+1}/{len(actions_list)}: {a}")
            # If LLM returned a simple control (not wrapped in mcp_action), coerce it
            if isinstance(a, dict) and "mcp_action" not in a and ("device" in a or "action" in a):
                # coerce to mcp_action control_device if it looks like a simple control
                coerced = {"mcp_action": "control_device", "parameters": [a]}
                await handle_action(coerced)
            else:
                await handle_action(a)
        except Exception as e:
            print(f"❌ Error executing action {a}: {e}")
            # continue with next actions

# ----------------------------
# Deepgram audio loop (silence frames while speaking)
# ----------------------------
DG_URL = "wss://api.deepgram.com/v1/listen?model=nova-2&language=en-US&encoding=linear16&sample_rate=16000&smart_format=true&keepalive=true"

async def deepgram_loop():
    global is_awake, command_buffer, microphone_active

    print("🎙️ Connecting to Deepgram...")
    async with websockets.connect(DG_URL, extra_headers={"Authorization": f"Token {DEEPGRAM_KEY}"}) as ws:
        print("✅ Listening... Say 'Hey Robert'!")
        loop = asyncio.get_running_loop()

        def audio_callback(indata, frames, time_info, status):
            global microphone_active
            if status:
                print(status)
            try:
                if microphone_active:
                    audio = (indata * 32767).astype(np.int16).tobytes()
                else:
                    silence = np.zeros_like(indata, dtype=np.float32)
                    audio = (silence * 32767).astype(np.int16).tobytes()
                asyncio.run_coroutine_threadsafe(ws.send(audio), loop)
            except Exception as e:
                print(f"⚠️ Audio send error: {e}")

        stream = sd.InputStream(callback=audio_callback, channels=1, samplerate=16000, dtype="float32")
        stream.start()

        try:
            async for message in ws:
                data = json.loads(message)
                if "channel" not in data:
                    continue

                transcript = data["channel"]["alternatives"][0]["transcript"].lower().strip()
                is_final = data.get("is_final", False)
                if not transcript:
                    continue
                print(f"🗣️ Heard: {transcript}")

                # Wake Word Detection
                if not is_awake and any(w in transcript for w in WAKE_WORDS):
                    is_awake = True
                    command_buffer = ""
                    print("🟢 Wake word detected!")
                    # Acknowledge via TTS (verbal confirmation)
                    await speak("Yes?")
                    continue

                # Capture command while awake
                if is_awake:
                    command_buffer += " " + transcript
                    if is_final:
                        full_command = command_buffer.strip()
                        if full_command:
                            print(f"💬 Processing: {full_command}")
                            result = ask_ollama(full_command)
                            # If ask_ollama returns a raw string or fails, guard it
                            if not isinstance(result, dict):
                                print("⚠️ LLM result not a dict, skipping execute.")
                            else:
                                await execute_logic(result)
                        is_awake = False
                        command_buffer = ""
                        print("💤 Waiting for wake word...")
        finally:
            stream.stop()
            stream.close()

# ----------------------------
# MAIN
# ----------------------------
if __name__ == "__main__":
    try:
        asyncio.run(deepgram_loop())
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user.")
