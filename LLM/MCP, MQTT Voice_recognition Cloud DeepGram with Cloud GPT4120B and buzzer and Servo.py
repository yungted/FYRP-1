#!/usr/bin/env python3
"""
PC-side script:
- Streams microphone audio to Deepgram for realtime transcription.
- Detects wake word, sends the captured command to Ollama LLM.
- Fetches the Pi device registry (subscribed via MQTT) and injects it into the system prompt so the LLM knows available devices.
- Validates the returned MCP JSON, then publishes to the Pi over MQTT (robot/control).
- Subscribes to robot/status to receive status updates from the Pi.

Prereqs:
pip install websockets sounddevice numpy paho-mqtt ollama
(make sure ollama client is available and configured
"""

import asyncio
import json
import os
import re
import time
import websockets
import sounddevice as sd
import numpy as np
import paho.mqtt.client as mqtt
from ollama import Client

# =========================
# CONFIGURATION (edit as needed)
# =========================
DEEPGRAM_KEY = "dc36253a111632480faebd24920862ea1265df6a"
OLLAMA_KEY = "add508d2d17443e8b9953322d3751470.vmwVMnfKMep0zN42BPIMoidp"

# MQTT broker address (where mosquitto is running)
MQTT_BROKER = "192.168.0.165"

# Deepgram websocket URL (realtime speech-to-text)
DG_URL = (
    "wss://api.deepgram.com/v1/listen?"
    "model=nova-2&language=en-US&encoding=linear16&sample_rate=16000&smart_format=true"
)

# MQTT topics (convention)
CONTROL_TOPIC = "robot/control"      # -> Pi will subscribe to this for control commands
STATUS_TOPIC = "robot/status"        # <- Pi publishes status here
REQUEST_TOPIC = "robot/request"      # -> general request topic (e.g., "status")
REGISTRY_TOPIC = "robot/registry"    # <- Pi publishes its capabilities here (retained)

# Wake words that begin a command
WAKE_WORDS = ["hey robert", "hey, robert", "hi robert", "okay robert"]
is_awake = False
command_buffer = ""

# Where we save the last-known device registry
DEVICES_FILE = "devices.json"

# =========================
# MQTT Setup
# =========================
mqtt_client = mqtt.Client(client_id="pc-llm-client")

def on_connect(client, userdata, flags, rc):
    """
    Called when the PC connects to the MQTT broker.
    Subscribe to STATUS and REGISTRY so we get Pi information.
    """
    print(f"✅ MQTT connected ({MQTT_BROKER}) rc={rc}")
    client.subscribe(STATUS_TOPIC)
    client.subscribe(REGISTRY_TOPIC)
    # Request a status update (Pi should respond)
    client.publish(REQUEST_TOPIC, json.dumps({"request": "status"}))

def on_message(client, userdata, msg):
    """
    Handle incoming MQTT messages:
    - Status messages from the Pi (robot/status)
    - Registry messages from the Pi (robot/registry) — save them locally
    """
    try:
        payload = json.loads(msg.payload.decode())
    except Exception as e:
        print("⚠️ MQTT message parse error:", e)
        return

    if msg.topic == STATUS_TOPIC:
        # Simple status log
        print(f"📥 STATUS: {payload}")

    elif msg.topic == REGISTRY_TOPIC:
        # Save the registry to disk for the LLM prompt builder
        print("📦 Device registry received from Pi:")
        print(json.dumps(payload, indent=2))
        try:
            with open(DEVICES_FILE, "w") as f:
                json.dump(payload, f, indent=2)
            print(f"💾 Saved registry to {DEVICES_FILE}")
        except Exception as e:
            print("⚠️ Failed to save registry:", e)

    else:
        # Other topics — log for debugging
        print(f"📫 {msg.topic}: {payload}")

# attach callbacks and connect
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)
mqtt_client.loop_start()  # run MQTT network loop in background thread

def publish(msg: dict):
    """Publish control messages to the Pi via MQTT (robot/control)."""
    try:
        mqtt_client.publish(CONTROL_TOPIC, json.dumps(msg))
        print(f"📤 Sent → {msg}")
    except Exception as e:
        print("⚠️ Publish failed:", e)

# =========================
# Note frequency helpers (for buzzer note names)
# =========================
NOTE_FREQS = {
    "C4": 261.63, "C#4": 277.18, "DB4": 277.18, "D4": 293.66, "D#4": 311.13, "EB4": 311.13,
    "E4": 329.63, "F4": 349.23, "F#4": 369.99, "GB4": 369.99, "G4": 392.00, "G#4": 415.30,
    "AB4": 415.30, "A4": 440.00, "A#4": 466.16, "BB4": 466.16, "B4": 493.88,
    "C5": 523.25, "D5": 587.33, "E5": 659.25
}

def note_to_freq(note: str):
    """Convert note name to frequency if present in the table."""
    if not note:
        return None
    n = note.strip().upper().replace("♯", "#").replace("＃", "#")
    return NOTE_FREQS.get(n)

# =========================
# Ollama (LLM) Setup
# =========================
os.environ["OLLAMA_API_KEY"] = OLLAMA_KEY
ollama_client = Client(
    host="https://ollama.com",
    headers={"Authorization": f"Bearer {OLLAMA_KEY}"}
)

def load_devices_for_prompt():
    """
    Read the saved devices.json (published by the Pi) and return a string
    describing device capabilities for the LLM system prompt.
    """
    try:
        with open(DEVICES_FILE, "r") as f:
            obj = json.load(f)
    except FileNotFoundError:
        return "(No device registry available.)\n"
    except Exception:
        return "(Error reading device registry.)\n"

    # Create a short human-readable summary of available outputs
    lines = []
    # The Pi may publish a dict with "outputs" directly or a full registry object
    outputs = obj.get("outputs") if isinstance(obj, dict) and obj.get("outputs") else None
    if outputs:
        for o in outputs:
            name = o.get("name", "unknown")
            typ = o.get("type", "unknown")
            actions = o.get("actions", [])
            extra = {k: v for k, v in o.items() if k not in ("name", "type", "actions")}
            lines.append(f"- {name} (type: {typ}, actions: {actions}, extra: {extra})")
    else:
        # If the Pi published a top-level registry with device_id + outputs
        # attempt to find "outputs" inside
        if isinstance(obj, dict) and any(isinstance(v, list) for v in obj.values()):
            for k, v in obj.items():
                if isinstance(v, list):
                    lines.append(f"{k}:")
                    for item in v:
                        lines.append(f"  - {item}")
        else:
            lines.append(str(obj))

    return "\n".join(lines)

def build_system_message():
    """
    Build the system message for Ollama that contains:
    - the required MCP schema
    - the current device capability summary from the Pi
    """
    schema_msg = (
        "You are an IoT controller that MUST output exactly one JSON object following the MCP schema.\n"
        "Schema: {\"mcp_action\": \"control_device\", \"execution\": \"sequential|parallel\", \"parameters\": [ {...} ] }\n"
        "Each parameter object must include a 'device' field (string) and action-specific fields.\n"
        "Allowed device examples: LED (state:on/off/blink), BUZZER (action:beep or 'sequence' list), SERVO (angle 0-180).\n"
        "You MUST only reference devices that exist in the available device list below.\n"
        "Respond with JSON only (no explanation)."
    )
    devices_summary = load_devices_for_prompt()
    system = schema_msg + "\n\nAvailable devices (from Pi):\n" + devices_summary
    return system

def ask_ollama(prompt: str):
    """
    Send the user's command to Ollama with the system prompt containing device info.
    Extract and return the first JSON object found in the model response.
    """
    system = build_system_message()
    try:
        res = ollama_client.chat(
            model="gpt-oss:120b-cloud",
            messages=[
                {"role": "system", "content": system},
                {"role": "user", "content": prompt},
            ],
            stream=False,
        )
        text = res["message"]["content"]
        print("🤖 LLM raw reply:\n", text)
        # extract first JSON object/array found
        m = re.search(r'(\{[\s\S]*\}|\[[\s\S]*\])', text)
        if not m:
            print("⚠️ LLM did not return JSON.")
            return {}
        parsed = json.loads(m.group())
        return parsed
    except Exception as e:
        print("⚠️ LLM failed:", e)
        return {}

# =========================
# Validation helpers
# =========================
def validate_mcp(mcp: dict):
    """
    Very basic validation:
    - mcp_action must be 'control_device'
    - parameters must be a list and devices referenced must be known from devices.json
    Returns (True, None) if OK, else (False, reason)
    """
    try:
        if mcp.get("mcp_action") != "control_device":
            return False, "mcp_action must be 'control_device'"
        params = mcp.get("parameters")
        if not isinstance(params, list):
            return False, "parameters must be a list"

        # load device names known from the registry (lowercased for comparison)
        known = set()
        try:
            with open(DEVICES_FILE, "r") as f:
                reg = json.load(f)
                for o in reg.get("outputs", []):
                    known.add(o.get("name", "").lower())
        except Exception:
            pass  # no registry -> be permissive

        if known:
            for p in params:
                dev = (p.get("device") or "").lower()
                if dev not in known:
                    return False, f"device '{dev}' not in registry"
        return True, None
    except Exception as e:
        return False, f"validation exception: {e}"

# =========================
# ACTION EXECUTION (async)
# same as your original logic but keeps publish() as the transport
# =========================
async def execute_action(action: dict):
    """Execute one action (LED, BUZZER, SERVO, etc.) with support for multiple devices."""
    try:
        dev_raw = action.get("device") or ""
        dev = dev_raw.upper()
    except Exception:
        print("⚠️ Invalid action format:", action)
        return

    # --- LED ---
    if dev.startswith("LED"):
        state = (action.get("state") or action.get("action") or "off").lower()
        if state == "blink":
            times = int(action.get("times", 3))
            delay = float(action.get("delay", 0.25))
            for _ in range(times):
                publish({"device": dev_raw, "state": "on"})
                await asyncio.sleep(delay)
                publish({"device": dev_raw, "state": "off"})
                await asyncio.sleep(delay)
        else:
            publish({"device": dev_raw, "state": state})
        return

    # --- BUZZER ---
    if dev.startswith("BUZZER"):
        seq_keys = ["sequence", "tone_sequence", "notes", "melody", "tones"]
        seq = None
        for k in seq_keys:
            if k in action:
                seq = action[k]
                break
        if isinstance(seq, list) and seq:
            for t in seq:
                if not isinstance(t, dict):
                    continue
                note = t.get("note") or t.get("pitch")
                freq = (
                    t.get("freq")
                    or t.get("frequency_hz")
                    or t.get("frequency")
                    or note_to_freq(note)
                )
                if not freq:
                    freq = 440.0
                dur = (
                    t.get("duration_s")
                    or (t.get("duration_ms") / 1000 if t.get("duration_ms") else None)
                    or (t.get("duration") / 1000 if t.get("duration") and t.get("duration") > 5 else t.get("duration"))
                    or 0.5
                )
                dur = float(dur)
                publish({"device": dev_raw, "action": "buzz", "frequency": freq, "duration": dur})
                await asyncio.sleep(dur + 0.05)
            return
        # fallback single buzz
        freq = float(action.get("frequency", 2000))
        dur = float(action.get("duration", 0.5))
        publish({"device": dev_raw, "action": "buzz", "frequency": freq, "duration": dur})
        await asyncio.sleep(dur + 0.05)
        return

    # --- SERVO ---
    if dev.startswith("SERVO"):
        angle = action.get("angle") or action.get("parameters", {}).get("angle", 90)
        try:
            angle = float(angle)
        except Exception:
            angle = 90.0
        publish({"device": dev_raw, "angle": angle})
        await asyncio.sleep(0.3)
        return

    # --- Unknown device ---
    print("⚠️ Unknown device in action:", dev_raw)

async def execute_mcp(mcp: dict):
    """
    Execute MCP structured command:
    { "mcp_action":"control_device", "execution":"sequential|parallel", "parameters":[...] }
    """
    if not mcp:
        print("⚠️ Empty MCP command")
        return

    # normalize parameter list robustly
    actions = None
    if isinstance(mcp.get("parameters"), list):
        actions = mcp.get("parameters")
    elif isinstance(mcp.get("parameters"), dict):
        actions = [mcp.get("parameters")]
    elif mcp.get("device") or mcp.get("action"):
        actions = [mcp]
    else:
        for v in mcp.values():
            if isinstance(v, list):
                actions = v
                break

    if not actions:
        print("⚠️ No actionable parameters found in MCP:", mcp)
        return

    exec_mode = (mcp.get("execution") or mcp.get("mode") or "sequential").lower()
    print(f"🧩 MCP execution={exec_mode}, actions={len(actions)}")

    normalized = [a for a in actions if isinstance(a, dict)]

    if exec_mode == "parallel":
        await asyncio.gather(*(execute_action(a) for a in normalized))
    else:
        for a in normalized:
            await execute_action(a)

# =========================
# Deepgram streaming (captures microphone audio and receives transcripts)
# =========================
async def deepgram_main():
    global is_awake, command_buffer

    async with websockets.connect(
        DG_URL, extra_headers={"Authorization": f"Token {DEEPGRAM_KEY}"}
    ) as ws:
        print("🎙️ Say 'Hey Robert' to start...")

        loop = asyncio.get_running_loop()

        def callback(indata, frames, time_info, status):
            # Convert float audio to int16 bytes and send to Deepgram websocket
            try:
                audio = (indata * 32767).astype(np.int16).tobytes()
                asyncio.run_coroutine_threadsafe(ws.send(audio), loop)
            except Exception as e:
                print("⚠️ audio callback error:", e)

        async def send_audio():
            # Open default microphone input and keep sending audio
            with sd.InputStream(callback=callback, channels=1, samplerate=16000, dtype="float32"):
                while True:
                    await asyncio.sleep(0.1)

        async def receive_transcripts():
            global is_awake, command_buffer
            async for message in ws:
                try:
                    data = json.loads(message)
                except Exception:
                    continue

                if "channel" not in data:
                    continue

                alt = data["channel"]["alternatives"][0]
                transcript = alt.get("transcript", "").lower().strip()
                is_final = data.get("is_final", False)

                if not transcript:
                    continue

                print(f"🗣️ {transcript}")

                # Detect wake word
                if not is_awake and any(w in transcript for w in WAKE_WORDS):
                    is_awake = True
                    command_buffer = ""
                    print("🟢 Wake word detected! Listening for command...")
                    # small audio cue on Pi (optional)
                    publish({"device": "BUZZER", "action": "beep"})
                    continue

                if is_awake:
                    # Accumulate interim transcripts until final
                    command_buffer += " " + transcript

                    if is_final:
                        cleaned = command_buffer.strip()
                        if cleaned:
                            print(f"💬 Command received: {cleaned}")
                            # Send to LLM and get MCP JSON
                            mcp = ask_ollama(cleaned)
                            if not mcp:
                                print("⚠️ No MCP returned by LLM.")
                            else:
                                # Validate before execution
                                ok, reason = validate_mcp(mcp)
                                if not ok:
                                    print("⚠️ MCP validation failed:", reason)
                                else:
                                    try:
                                        await execute_mcp(mcp)  # executes by publishing to MQTT
                                    except Exception as e:
                                        print("⚠️ execute_mcp failed:", e)
                        else:
                            print("⚠️ No command detected.")
                        print("\n🎤 Say 'Hey Robert' to start again...")
                        is_awake = False
                        command_buffer = ""

        await asyncio.gather(send_audio(), receive_transcripts())

# =========================
# MAIN
# =========================
if __name__ == "__main__":
    try:
        asyncio.run(deepgram_main())
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user.")
