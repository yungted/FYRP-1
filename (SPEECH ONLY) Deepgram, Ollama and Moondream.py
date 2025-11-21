#!/usr/bin/env python3
"""
Voice-controlled camera + Ollama reasoning via Deepgram ASR + Moondream.
- Say wake word (e.g., "hey robert") to start listening
- Ollama decides whether to capture image or control devices
- Moondream analyzes captured image if requested
Prereqs:
pip install websockets sounddevice numpy paho-mqtt ollama Pillow opencv-python moondream
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
from datetime import datetime
import cv2
from PIL import Image
import moondream as md

# ----------------------------
# CONFIGURATION
# ----------------------------
DEEPGRAM_KEY = "dc36253a111632480faebd24920862ea1265df6a"
OLLAMA_KEY = "add508d2d17443e8b9953322d3751470.vmwVMnfKMep0zN42BPIMoidp"
MOONDREAM_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJrZXlfaWQiOiIxZWI5Njk0MS02MDVlLTQ3ZTYtYjI5NS01NjdjMGRjYWZhYTkiLCJvcmdfaWQiOiJlZzJ2bXo4a1FWY0xlOTExQVl5bkM0NU44YkhEYTMwTyIsImlhdCI6MTc2MjkzMzIzOSwidmVyIjoxfQ.onaMxtqh-X14ggd8XUDfiALxzYuHuUyI3GMbWHBYCqw"

CAM_INDEX = 2  # Default webcam
WAKE_WORDS = ["hey robert", "hey, robert", "hi robert", "okay robert"]

is_awake = False
command_buffer = ""
DEVICES_FILE = "devices.json"

# MQTT Setup
MQTT_BROKER = "192.168.0.165"
CONTROL_TOPIC = "robot/control"
STATUS_TOPIC = "robot/status"
REQUEST_TOPIC = "robot/request"
REGISTRY_TOPIC = "robot/registry"

mqtt_client = mqtt.Client(client_id="pc-llm-client")

# Ollama client
os.environ["OLLAMA_API_KEY"] = OLLAMA_KEY
ollama_client = Client(host="https://ollama.com", headers={"Authorization": f"Bearer {OLLAMA_KEY}"})

# Moondream client
moondream_model = md.vl(endpoint="https://api.moondream.ai/v1", api_key=MOONDREAM_KEY)

# ----------------------------
# MQTT Callbacks
# ----------------------------
def on_connect(client, userdata, flags, rc):
    print(f"✅ MQTT connected ({MQTT_BROKER}) rc={rc}")
    client.subscribe(STATUS_TOPIC)
    client.subscribe(REGISTRY_TOPIC)
    client.publish(REQUEST_TOPIC, json.dumps({"request": "status"}))

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        return
    if msg.topic == REGISTRY_TOPIC:
        try:
            with open(DEVICES_FILE, "w") as f:
                json.dump(payload, f, indent=2)
            print("💾 Saved registry from Pi")
        except Exception as e:
            print("⚠️ Failed saving registry:", e)
    else:
        print(f"📥 {msg.topic}: {payload}")

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)
mqtt_client.loop_start()

def publish(msg: dict):
    try:
        mqtt_client.publish(CONTROL_TOPIC, json.dumps(msg))
        print(f"📤 Sent → {msg}")
    except Exception as e:
        print("⚠️ Publish failed:", e)

# ----------------------------
# Camera & Moondream
# ----------------------------
def capture_image(filename: str = "capture.jpg") -> str:
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("⚠️ Cannot open camera")
        return None
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("⚠️ Failed to capture frame")
        return None
    cv2.imwrite(filename, frame)
    print(f"📸 Captured → {filename}")
    return filename

def analyze_with_moondream(filename: str):
    print("🔍 Sending image to Moondream...")
    try:
        image = Image.open(filename)
        response = moondream_model.query(image, "Describe this image.")
        answer = response.get("answer", "No description returned")
        print("✅ Vision result:", answer)
    except Exception as e:
        print("⚠️ Moondream error:", e)

# ----------------------------
# Ollama helpers
# ----------------------------
def load_devices_for_prompt():
    try:
        with open(DEVICES_FILE, "r") as f:
            obj = json.load(f)
    except Exception:
        return "(No device registry)\n"
    lines = []
    outputs = obj.get("outputs") if isinstance(obj, dict) else None
    if outputs:
        for o in outputs:
            name = o.get("name", "unknown")
            typ = o.get("type", "unknown")
            actions = o.get("actions", [])
            lines.append(f"- {name} (type:{typ}, actions:{actions})")
    return "\n".join(lines)

def build_system_message():
    schema_msg = (
        "You are an IoT controller. MUST respond with exactly one JSON object.\n"
        "Schema examples:\n"
        "  Control devices: {\"mcp_action\":\"control_device\",\"execution\":\"sequential\",\"parameters\":[{...}]}\n"
        "  Vision request: {\"mcp_action\":\"capture_image\",\"parameters\":{}}\n"
        "If the user asks about surroundings or 'what's in front', respond with capture_image.\n"
        "Respond with JSON only, no explanation.\n"
    )
    devices_summary = load_devices_for_prompt()
    return schema_msg + "\nAvailable devices:\n" + devices_summary

def ask_ollama(prompt: str):
    system = build_system_message()
    try:
        res = ollama_client.chat(
            model="deepseek-v3.1:671b-cloud",
            messages=[{"role":"system","content":system},{"role":"user","content":prompt}],
            stream=False
        )
        text = res["message"]["content"]
        print("🤖 LLM raw reply:\n", text)
        m = re.search(r'(\{[\s\S]*\}|\[[\s\S]*\])', text)
        if m:
            return json.loads(m.group())
    except Exception as e:
        print("⚠️ LLM failed:", e)
    return {}

# ----------------------------
# MCP execution
# ----------------------------
async def execute_mcp(mcp: dict):
    if not mcp:
        return
    action_type = mcp.get("mcp_action")
    if action_type == "capture_image":
        filename = capture_image()
        if filename:
            analyze_with_moondream(filename)
        return
    if action_type != "control_device":
        return
    # Execute control_device as before
    params = mcp.get("parameters", [])
    for p in params:
        await execute_action(p)

async def execute_action(action: dict):
    dev_raw = action.get("device") or ""
    dev = dev_raw.upper()

    # --- LED ---
    if dev.startswith("LED"):
        # Accept either 'state' or 'action' for LED
        state = (action.get("state") or action.get("action") or "off").lower()
        if state not in ["on", "off", "blink"]:
            state = "off"
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
        # Simple beep if 'action' is beep or unspecified
        act = (action.get("action") or "").lower()
        if act == "beep" or act == "":
            freq = float(action.get("frequency", 2000))
            dur = float(action.get("duration", 0.5))
            publish({"device": dev_raw, "action": "buzz", "frequency": freq, "duration": dur})
            await asyncio.sleep(dur + 0.05)
            return

        # Handle sequences
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

    print("⚠️ Unknown device or action:", dev_raw, action)

# ----------------------------
# Deepgram streaming
# ----------------------------
DG_URL = (
    "wss://api.deepgram.com/v1/listen?"
    "model=nova-2&language=en-US&encoding=linear16&sample_rate=16000&smart_format=true"
)

async def deepgram_main():
    global is_awake, command_buffer

    async with websockets.connect(DG_URL, extra_headers={"Authorization": f"Token {DEEPGRAM_KEY}"}) as ws:
        print("🎙️ Say 'Hey Robert' to start...")
        loop = asyncio.get_running_loop()

        def callback(indata, frames, time_info, status):
            try:
                audio = (indata * 32767).astype(np.int16).tobytes()
                asyncio.run_coroutine_threadsafe(ws.send(audio), loop)
            except Exception as e:
                print("⚠️ audio callback:", e)

        async def send_audio():
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

                if not is_awake and any(w in transcript for w in WAKE_WORDS):
                    is_awake = True
                    command_buffer = ""
                    print("🟢 Wake word detected! Listening...")
                    publish({"device": "BUZZER", "action": "beep"})
                    continue

                if is_awake:
                    command_buffer += " " + transcript
                    if is_final:
                        cleaned = command_buffer.strip()
                        if cleaned:
                            print(f"💬 Command: {cleaned}")
                            mcp = ask_ollama(cleaned)
                            if mcp:
                                await execute_mcp(mcp)
                        is_awake = False
                        command_buffer = ""
                        print("\n🎤 Say 'Hey Robert' to start again...")

        await asyncio.gather(send_audio(), receive_transcripts())

# ----------------------------
# MAIN
# ----------------------------
if __name__ == "__main__":
    try:
        asyncio.run(deepgram_main())
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user.")
