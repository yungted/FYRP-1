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
    # Request status/devices on connect so Pi can respond (retained + active)
    client.publish(REQUEST_TOPIC, json.dumps({"request": "status"}))
    client.publish(REQUEST_TOPIC, json.dumps({"request": "devices"}))

def on_message(client, userdata, msg):
    global device_states, device_registry
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        print("⚠️ Non-JSON MQTT payload on", msg.topic)
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
def ask_ollama(user_text):
    device_list = load_devices_prompt()
    device_status = get_device_status_prompt()
    system_prompt = (
        "You are Robert, a helpful intelligent assistant.\n"
        "You have access to IoT devices and a camera.\n\n"
        "DEVICE LIST (from devices.json):\n"
        f"{device_list}\n\n"
        "CURRENT DEVICE STATES (live telemetry):\n"
        f"{device_status}\n\n"
        "User Request Handling:\n"
        "1. 'reply': A short, friendly verbal response for the user.\n"
        "2. 'action': Either a single action object or a list of actions. Each action must be a JSON object containing 'mcp_action' and optional 'parameters'.\n\n"
        "Action Schema Examples:\n"
        "- Control Device: {\"mcp_action\":\"control_device\",\"parameters\":[{\"device\":\"LED\",\"state\":\"on\"}]}\n"
        "- Delay: {\"mcp_action\":\"delay\",\"parameters\":[{\"duration_seconds\":3}]}\n"
        "- Vision: {\"mcp_action\":\"capture_image\",\"parameters\":[{\"vision_prompt\":\"Count the fingers visible.\"}]}\n\n"
        "IMPORTANT: Ensure 'device' fields match available device ids/names listed above. If multiple steps are needed, return 'action' as a list of action objects.\n"
        "Example Response: {\"reply\":\"OK\",\"action\":[{\"mcp_action\":\"control_device\",\"parameters\":[{\"device\":\"LED\",\"state\":\"on\"}]},{\"mcp_action\":\"delay\",\"parameters\":[{\"duration_seconds\":3}]},{\"mcp_action\":\"control_device\",\"parameters\":[{\"device\":\"LED\",\"state\":\"off\"}]}]}"
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
    We'll validate each param and publish using publish_control_message().
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
    # find first vision_prompt in parameters
    prompt = None
    if isinstance(parameters, list):
        for p in parameters:
            if p.get("vision_prompt"):
                prompt = p.get("vision_prompt")
                break
    elif isinstance(parameters, dict):
        prompt = parameters.get("vision_prompt")
    if not prompt:
        prompt = "Describe this image in detail."
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
