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
import websockets
import sounddevice as sd
import numpy as np
import paho.mqtt.client as mqtt
from ollama import Client
import cv2
from PIL import Image
import moondream as md
from TTS.api import TTS

# ----------------------------
# CONFIGURATION
# ----------------------------
DEEPGRAM_KEY = "dc36253a111632480faebd24920862ea1265df6a"
OLLAMA_KEY = "add508d2d17443e8b9953322d3751470.vmwVMnfKMep0zN42BPIMoidp"
MOONDREAM_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJrZXlfaWQiOiIxZWI5Njk0MS02MDVlLTQ3ZTYtYjI5NS01NjdjMGRjYWZhYTkiLCJvcmdfaWQiOiJlZzJ2bXo4a1FWY0xlOTExQVl5bkM0NU44YkhEYTMwTyIsImlhdCI6MTc2MjkzMzIzOSwidmVyIjoxfQ.onaMxtqh-X14ggd8XUDfiALxzYuHuUyI3GMbWHBYCqw"

CAM_INDEX = 2  # Default webcam index
WAKE_WORDS = ["hey robert", "hey, robert", "hi robert", "okay robert"]
MQTT_BROKER = "192.168.0.165"

# Topics (subscribe to both legacy and MCP topics for compatibility)
CONTROL_TOPIC = "robot/control"
STATUS_TOPIC = "robot/status"
REQUEST_TOPIC = "robot/request"
REGISTRY_TOPIC_LEGACY = "robot/registry"
REGISTRY_TOPIC_MCP = "robot/device_registry"
DEVICES_FILE = "devices.json"

# ROS Command Topics
ROS_CMD_TOPIC = "robot/ros_cmd"          # Publish ROS commands here
ROS_CMD_VEL_TOPIC = "robot/ros_cmd_vel"  # Direct velocity commands
ROS_FEEDBACK_TOPIC = "robot/ros_feedback" # Receive feedback from Jetson

# ----------------------------
# SAFE ROS COMMAND CONFIGURATION
# ----------------------------
# Maximum safe velocities for service robot (m/s and rad/s)
MAX_LINEAR_VELOCITY = 0.5    # 0.5 m/s max forward/backward
MAX_ANGULAR_VELOCITY = 0.8   # 0.8 rad/s max rotation
DEFAULT_MOVE_DURATION = 2.0  # Default duration for movements (seconds)

# Safety limits for modular commands
MAX_DISTANCE_METERS = 5.0    # Maximum allowed distance in one command
MAX_DURATION_SECONDS = 15.0  # Maximum allowed duration in one command
MAX_ANGLE_DEGREES = 360.0    # Maximum rotation in one command

# ===========================================
# CALIBRATION FACTORS - ADJUST THESE VALUES!
# ===========================================
# If robot moves LESS than expected, INCREASE these values
# If robot moves MORE than expected, DECREASE these values
# Example: If "move 1 meter" only moves 0.5m, set DISTANCE_CALIBRATION = 2.0

DISTANCE_CALIBRATION = 1.0   # Multiplier for distance commands (try 1.5, 2.0, etc.)
DURATION_CALIBRATION = 1.0   # Multiplier for duration commands (try 1.2, 1.5, etc.)
ANGLE_CALIBRATION = 1.0      # Multiplier for rotation commands (try 1.5, 2.0, etc.)

# Acceleration compensation - adds extra time for robot to reach full speed
ACCEL_COMPENSATION_SEC = 0.3  # Extra seconds added to account for acceleration/deceleration

# Base velocities for modular commands (used for distance/angle calculations)
# These should match what your robot ACTUALLY achieves, not what you command
BASE_LINEAR_VELOCITY = 0.3   # m/s - Measure actual speed and update this!
BASE_ANGULAR_VELOCITY = 0.5  # rad/s - Measure actual rotation rate and update this!
SLOW_LINEAR_VELOCITY = 0.15  # m/s for slow movements

# Whitelisted safe commands with default parameters
# These serve as defaults when no distance/duration/angle is specified
# TIP: If defaults feel too short, increase the "duration" values here
SAFE_ROS_COMMANDS = {
    "move_forward": {"linear_x": 0.3, "angular_z": 0.0, "duration": 3.0, "type": "linear"},
    "move_backward": {"linear_x": -0.2, "angular_z": 0.0, "duration": 3.0, "type": "linear"},
    "turn_left": {"linear_x": 0.0, "angular_z": 0.5, "duration": 2.0, "type": "angular"},
    "turn_right": {"linear_x": 0.0, "angular_z": -0.5, "duration": 2.0, "type": "angular"},
    "stop": {"linear_x": 0.0, "angular_z": 0.0, "duration": 0.0, "type": "stop"},
    "slow_forward": {"linear_x": 0.15, "angular_z": 0.0, "duration": 4.0, "type": "linear"},
    "rotate_180": {"linear_x": 0.0, "angular_z": 0.5, "duration": 4.0, "type": "angular"},
    "slight_left": {"linear_x": 0.2, "angular_z": 0.2, "duration": 2.0, "type": "combined"},
    "slight_right": {"linear_x": 0.2, "angular_z": -0.2, "duration": 2.0, "type": "combined"},
    "approach": {"linear_x": 0.2, "angular_z": 0.0, "duration": 2.5, "type": "linear"},
    "retreat": {"linear_x": -0.15, "angular_z": 0.0, "duration": 2.5, "type": "linear"},
    "guide_forward": {"linear_x": 0.25, "angular_z": 0.0, "duration": 4.0, "type": "linear"},
}

# Emergency stop flag
emergency_stop_active = False

# ----------------------------
# GLOBAL OBJECTS
# ----------------------------
print("⏳ Loading TTS Model (this may take a moment)...")
tts_engine = TTS("tts_models/en/ljspeech/vits")
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
    # Request status/devices on connect so Pi can respond (retained + active)
    client.publish(REQUEST_TOPIC, json.dumps({"request": "status"}))
    client.publish(REQUEST_TOPIC, json.dumps({"request": "devices"}))

def on_message(client, userdata, msg):
    global device_states, device_registry, emergency_stop_active
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        print("⚠️ Non-JSON MQTT payload on", msg.topic)
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
    """
    Calculate duration needed to travel a given distance at a given velocity.
    Returns clamped duration within safety limits.
    """
    if velocity == 0:
        return 0
    # Clamp distance to safety limit
    safe_distance = max(0, min(MAX_DISTANCE_METERS, abs(distance_meters)))
    duration = safe_distance / abs(velocity)
    # Clamp duration to safety limit
    return min(duration, MAX_DURATION_SECONDS)

def calculate_duration_from_angle(angle_degrees, angular_velocity):
    """
    Calculate duration needed to rotate a given angle at a given angular velocity.
    Returns clamped duration within safety limits.
    """
    import math
    if angular_velocity == 0:
        return 0
    # Clamp angle to safety limit
    safe_angle = max(0, min(MAX_ANGLE_DEGREES, abs(angle_degrees)))
    angle_radians = math.radians(safe_angle)
    duration = angle_radians / abs(angular_velocity)
    # Clamp duration to safety limit
    return min(duration, MAX_DURATION_SECONDS)

def parse_modular_parameters(params, command_type):
    """
    Parse modular parameters (distance, duration, angle) from LLM output.
    Returns calculated duration based on the parameter type.
    
    Priority: duration > distance > angle > default
    Applies calibration factors for accuracy.
    """
    # If explicit duration is provided, use it (clamped) with calibration
    if "duration" in params:
        base_duration = max(0, min(MAX_DURATION_SECONDS, float(params["duration"])))
        calibrated = (base_duration * DURATION_CALIBRATION) + ACCEL_COMPENSATION_SEC
        print(f"⏱️ Duration {base_duration}s × {DURATION_CALIBRATION} + {ACCEL_COMPENSATION_SEC}s accel = {calibrated:.2f}s")
        return min(calibrated, MAX_DURATION_SECONDS)
    
    # If distance is provided (for linear movements)
    if "distance" in params or "distance_meters" in params:
        distance = float(params.get("distance") or params.get("distance_meters", 0))
        if command_type in ("linear", "combined"):
            velocity = BASE_LINEAR_VELOCITY
            # Use slower velocity for slow commands
            if "slow" in str(params.get("command", "")).lower():
                velocity = SLOW_LINEAR_VELOCITY
            # Apply calibration: more distance needed = longer duration
            calibrated_distance = distance * DISTANCE_CALIBRATION
            calculated = calculate_duration_from_distance(calibrated_distance, velocity)
            calculated += ACCEL_COMPENSATION_SEC  # Add acceleration compensation
            calculated = min(calculated, MAX_DURATION_SECONDS)
            print(f"📏 Distance {distance}m × {DISTANCE_CALIBRATION} = {calibrated_distance:.2f}m → {calculated:.2f}s (at {velocity} m/s)")
            return calculated
    
    # If angle is provided (for angular movements)
    if "angle" in params or "angle_degrees" in params:
        angle = float(params.get("angle") or params.get("angle_degrees", 0))
        if command_type in ("angular", "combined"):
            # Apply calibration: more angle needed = longer duration
            calibrated_angle = angle * ANGLE_CALIBRATION
            calculated = calculate_duration_from_angle(calibrated_angle, BASE_ANGULAR_VELOCITY)
            calculated += ACCEL_COMPENSATION_SEC  # Add acceleration compensation
            calculated = min(calculated, MAX_DURATION_SECONDS)
            print(f"🔄 Angle {angle}° × {ANGLE_CALIBRATION} = {calibrated_angle:.1f}° → {calculated:.2f}s")
            return calculated
    
    # No modular parameters - return None to use default
    return None

def publish_ros_command(command_name, custom_params=None):
    """
    Publish a validated ROS command to the Jetson via MQTT.
    Only whitelisted commands are allowed.
    
    Supports modular parameters:
    - duration: Direct duration in seconds (0-15s)
    - distance / distance_meters: Distance in meters (converted to duration)
    - angle / angle_degrees: Rotation angle in degrees (converted to duration)
    """
    global emergency_stop_active
    
    if emergency_stop_active and command_name != "stop":
        print("🛑 Emergency stop active - only 'stop' commands allowed")
        return False
    
    command_name = command_name.lower().strip()
    
    # Check if command is whitelisted
    if command_name not in SAFE_ROS_COMMANDS:
        print(f"⚠️ Unknown ROS command '{command_name}' - not in whitelist. Ignoring.")
        print(f"   Available commands: {list(SAFE_ROS_COMMANDS.keys())}")
        return False
    
    # Get base command parameters
    base_params = SAFE_ROS_COMMANDS[command_name].copy()
    command_type = base_params.get("type", "linear")
    
    # Parse modular parameters (distance, angle, duration)
    if custom_params and isinstance(custom_params, dict):
        # Add command to params for context
        custom_params["command"] = command_name
        calculated_duration = parse_modular_parameters(custom_params, command_type)
        if calculated_duration is not None:
            base_params["duration"] = calculated_duration
        # Do NOT allow velocity overrides from LLM - use predefined safe values only
    
    # Validate velocities (should already be safe, but double-check)
    linear_x, angular_z, _ = validate_ros_velocity(
        base_params.get("linear_x", 0.0),
        base_params.get("angular_z", 0.0)
    )
    
    # Calculate expected distance/angle for logging
    expected_distance = abs(linear_x) * base_params.get("duration", DEFAULT_MOVE_DURATION)
    expected_angle_rad = abs(angular_z) * base_params.get("duration", DEFAULT_MOVE_DURATION)
    import math
    expected_angle_deg = math.degrees(expected_angle_rad)
    
    ros_msg = {
        "command": command_name,
        "twist": {
            "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
        },
        "duration": base_params.get("duration", DEFAULT_MOVE_DURATION),
        "expected_distance_m": round(expected_distance, 2),
        "expected_rotation_deg": round(expected_angle_deg, 1),
        "timestamp": asyncio.get_event_loop().time() if asyncio.get_event_loop().is_running() else 0
    }
    
    mqtt_client.publish(ROS_CMD_TOPIC, json.dumps(ros_msg))
    
    # Enhanced logging
    if command_type == "linear" and expected_distance > 0:
        print(f"🤖 Published ROS command: {command_name} -> {expected_distance:.2f}m over {ros_msg['duration']:.2f}s")
    elif command_type == "angular" and expected_angle_deg > 0:
        print(f"🤖 Published ROS command: {command_name} -> {expected_angle_deg:.1f}° over {ros_msg['duration']:.2f}s")
    else:
        print(f"🤖 Published ROS command: {command_name} -> linear_x={linear_x}, angular_z={angular_z}, duration={ros_msg['duration']}s")
    
    return True

def emergency_stop():
    """Immediately stop the robot and set emergency flag."""
    global emergency_stop_active
    emergency_stop_active = True
    stop_msg = {
        "command": "emergency_stop",
        "twist": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        },
        "duration": 0,
        "emergency": True
    }
    mqtt_client.publish(ROS_CMD_TOPIC, json.dumps(stop_msg))
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
    """Return available ROS commands for the LLM with descriptions."""
    command_descriptions = {
        "move_forward": "Move forward (default ~0.6m, or specify distance/duration)",
        "move_backward": "Move backward (default ~0.4m, or specify distance/duration)",
        "turn_left": "Rotate left/counter-clockwise (default ~45°, or specify angle/duration)",
        "turn_right": "Rotate right/clockwise (default ~45°, or specify angle/duration)",
        "stop": "Immediately stop all movement",
        "slow_forward": "Move forward slowly (default ~0.45m, or specify distance/duration)",
        "rotate_180": "Rotate 180 degrees (turn around)",
        "slight_left": "Move forward while turning slightly left",
        "slight_right": "Move forward while turning slightly right",
        "approach": "Slowly approach forward (default ~0.3m)",
        "retreat": "Slowly back away (default ~0.2m)",
        "guide_forward": "Guide mode - steady forward (default ~0.75m)",
    }
    lines = []
    for cmd, desc in command_descriptions.items():
        lines.append(f"  - {cmd}: {desc}")
    return "\n".join(lines)

def ask_ollama(user_text):
    device_list = load_devices_prompt()
    device_status = get_device_status_prompt()
    ros_commands = get_ros_commands_prompt()
    system_prompt = (
    "You are Robert, a helpful intelligent service robot assistant.\n"
    "You have access to IoT devices, a camera, and ROBOT MOVEMENT controls.\n\n"
    "DEVICE LIST (from devices.json):\n"
    f"{device_list}\n\n"
    "CURRENT DEVICE STATES (live telemetry):\n"
    f"{device_status}\n\n"
    "AVAILABLE ROS MOVEMENT COMMANDS (ONLY use these exact names):\n"
    f"{ros_commands}\n\n"
    "=== RESPONSE FORMAT ===\n"
    "You MUST respond with valid JSON only. Format:\n"
    "{\"reply\": \"<verbal response>\", \"action\": <action or list of actions>}\n\n"
    "Each action must be: {\"mcp_action\": \"<type>\", \"parameters\": [<params>]}\n\n"
    "=== SAFETY-FIRST REASONING ===\n"
    "Before executing ANY movement command, you MUST evaluate:\n"
    "1. INTENT ANALYSIS: What does the user actually want to achieve?\n"
    "2. SAFETY CHECK: Is this request safe in the current context?\n"
    "3. APPROPRIATE RESPONSE: Choose the safest command that achieves the goal.\n\n"
    "SAFETY RULES (MANDATORY):\n"
    "- ONLY use commands from AVAILABLE ROS MOVEMENT COMMANDS above.\n"
    "- NEVER invent or hallucinate new movement commands.\n"
    "- NEVER specify velocity values - they are predefined for safety.\n"
    "- If user asks for something dangerous (e.g., 'go fast', 'maximum speed'), politely REFUSE.\n"
    "- If unsure about intent, ASK for clarification instead of guessing.\n"
    "- Use 'stop' IMMEDIATELY if user says: stop, halt, wait, danger, emergency, or indicates concern.\n"
    "- For large movements, confirm with user first or break into smaller steps.\n"
    "- When guiding people, use slow, predictable movements (slow_forward, guide_forward).\n\n"
    "INTENT INTERPRETATION EXAMPLES:\n"
    "- 'Come here' / 'Come to me' → approach (slow, careful approach)\n"
    "- 'Go away' / 'Back off' → retreat (back away from user)\n"
    "- 'Follow me' / 'Come with me' → guide_forward (steady paced following)\n"
    "- 'Turn around' → rotate_180\n"
    "- 'Face me' / 'Look at me' → turn_left or turn_right based on context\n"
    "- 'Go left' / 'Go right' → slight_left or slight_right (forward + turn)\n"
    "- 'Just turn' → turn_left or turn_right (rotation only)\n"
    "- 'Move a bit' / 'Nudge forward' → slow_forward\n"
    "- 'STOP!' / 'Freeze!' / 'Halt!' → stop (immediate)\n\n"
    "MULTI-STEP SEQUENCES:\n"
    "For complex navigation, return multiple actions in order:\n"
    "{\"action\": [{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"turn_left\"}]},\n"
    "             {\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"move_forward\"}]}]}\n\n"
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
    "For vision requests, use 'capture_image' with a contextual vision_prompt:\n"
    "{\"mcp_action\":\"capture_image\",\"parameters\":[{\"vision_prompt\":\"<question>\"}]}\n\n"
    "=== IOT DEVICES ===\n"
    "For buzzer: use 'frequency': <Hz>, 'duration': <seconds>\n"
    "For LED/servo: use 'state': 'on'/'off' or position values\n"
    "Check device_states to avoid redundant actions (e.g., don't turn on LED if already on).\n\n"
    "EXAMPLES:\n"
    "User: 'Move forward' → {\"reply\":\"Moving forward.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"move_forward\"}]}]}\n"
    "User: 'Move forward 2 meters' → {\"reply\":\"Moving forward 2 meters.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"move_forward\",\"distance\":2.0}]}]}\n"
    "User: 'Turn left 90 degrees' → {\"reply\":\"Turning left 90 degrees.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"turn_left\",\"angle\":90}]}]}\n"
    "User: 'Move backward for 3 seconds' → {\"reply\":\"Moving backward for 3 seconds.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"move_backward\",\"duration\":3.0}]}]}\n"
    "User: 'Come here slowly' → {\"reply\":\"Approaching you carefully.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"approach\"}]}]}\n"
    "User: 'Go fast!' → {\"reply\":\"I'm sorry, I can only move at safe speeds for everyone's safety. Would you like me to move forward at my normal pace?\",\"action\":[]}\n"
    "User: 'Turn around and go forward 1 meter' → {\"reply\":\"Turning around, then moving forward 1 meter.\",\"action\":[{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"rotate_180\"}]},{\"mcp_action\":\"ros_command\",\"parameters\":[{\"command\":\"move_forward\",\"distance\":1.0}]}]}\n"
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
            available = ", ".join(SAFE_ROS_COMMANDS.keys())
            await speak(f"I can't do that movement. Available commands are: {available}")
        else:
            # Wait briefly for command to be received before next command
            await asyncio.sleep(0.1)

# central dispatcher for action types
async def handle_action(action_obj):
    """
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
                    # Acknowledge via buzzer (best-effort)
                    publish_control_message({"device": "buzzer", "action": "beep"})
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
