#!/usr/bin/env python3
import asyncio
import json
import os
import re
import websockets
import sounddevice as sd
import numpy as np
import paho.mqtt.client as mqtt
from ollama import Client

# ==========================
# CONFIGURATION
# ==========================

DEEPGRAM_API_KEY = "dc36253a111632480faebd24920862ea1265df6a"
DG_URL = (
    "wss://api.deepgram.com/v1/listen?"
    "model=nova-2&language=en-US&encoding=linear16&sample_rate=16000&smart_format=true"
)
WAKE_WORD = "hey robert"
GRACE_DELAY = 2
OLLAMA_API_KEY = "add508d2d17443e8b9953322d3751470.vmwVMnfKMep0zN42BPIMoidp"

# MQTT configuration
MQTT_BROKER = "192.168.0.165"
CONTROL_TOPIC = "robot/control"
STATUS_TOPIC = "robot/status"
REQUEST_TOPIC = "robot/request"

# ==========================
# GLOBAL STATE
# ==========================

led_state = "unknown"
is_listening_for_command = False
command_text = ""

# ==========================
# MQTT SETUP
# ==========================

def on_connect(client, userdata, flags, rc):
    print(f"\n✅ Connected to MQTT broker ({MQTT_BROKER}) with code {rc}")
    client.subscribe(STATUS_TOPIC)
    print(f"📡 Subscribed to {STATUS_TOPIC}")
    print("📨 Requesting LED state from Pi...")
    client.publish(REQUEST_TOPIC, json.dumps({"device": "LED", "request": "status"}))


def on_message(client, userdata, msg):
    global led_state
    try:
        payload = json.loads(msg.payload.decode())
        if payload.get("device") == "LED":
            led_state = payload.get("state", "unknown")
            print(f"💡 LED status updated from Pi → {led_state}")
    except Exception as e:
        print("⚠️ Error parsing MQTT message:", e)


mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)
mqtt_client.loop_start()

# ==========================
# OLLAMA CLIENT
# ==========================

os.environ["OLLAMA_API_KEY"] = OLLAMA_API_KEY
ollama_client = Client(
    host="https://ollama.com",
    headers={"Authorization": f"Bearer {OLLAMA_API_KEY}"}
)


def query_ollama(prompt: str) -> str:
    """Ask Ollama Cloud model for command parsing or reasoning."""
    try:
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a smart IoT control assistant that interprets natural language "
                    "to generate structured JSON commands for devices. "
                    "Available devices: LED, BUZZER, SERVO. "
                    "SERVO accepts an angle between 0 and 180 degrees, controlling rotation. "
                    "Example: {\"device\": \"SERVO\", \"angle\": 90}. "
                    "Note that 90 degrees is North, 0 degrees is east, 180 degrees is west."
                    "You can combine or sequence actions logically."
                ),
            },
            {"role": "user", "content": prompt},
        ]
        response = ollama_client.chat(model="gpt-oss:120b-cloud", messages=messages, stream=False)
        return response["message"]["content"]
    except Exception as e:
        return f"⚠️ Ollama Cloud request failed: {e}"

# ==========================
# DEVICE CONTROL FUNCTIONS
# ==========================

def send_led_command(state: str):
    msg = json.dumps({"device": "LED", "pin": 3, "state": state})
    mqtt_client.publish(CONTROL_TOPIC, msg)
    print(f"📤 Sent LED command → {msg}")


def send_buzzer_command(frequency=2000, duration=0.5):
    msg = json.dumps({
        "device": "BUZZER",
        "action": "buzz",
        "frequency": frequency,
        "duration": duration
    })
    mqtt_client.publish(CONTROL_TOPIC, msg)
    print(f"📤 Sent buzzer command → {msg}")


def send_servo_command(angle: float):
    """Send servo angle command to Raspberry Pi."""
    msg = json.dumps({"device": "SERVO", "angle": angle})
    mqtt_client.publish(CONTROL_TOPIC, msg)
    print(f"📤 Sent SERVO command → {msg}")


async def send_buzzer_beep():
    tones = [
        {"frequency": 261.63, "duration": 0.1},
        {"frequency": 392.00, "duration": 0.3},
    ]
    for tone in tones:
        msg = json.dumps({
            "device": "BUZZER",
            "action": "buzz",
            "frequency": tone["frequency"],
            "duration": tone["duration"]
        })
        mqtt_client.publish(CONTROL_TOPIC, msg)
        await asyncio.sleep(0.03)
    print("🎵 Played wake tone")


def request_led_status():
    mqtt_client.publish(REQUEST_TOPIC, json.dumps({"device": "LED", "request": "status"}))
    print("📨 Requested LED status from Pi...")

# ==========================
# COMMAND HANDLER
# ==========================

def handle_command(command: str):
    prompt = f"""
User said: "{command}"

You have control over:

* LED: 'on', 'off', or 'status'.
* BUZZER: 'beep' once or 'buzz' for X seconds at Y Hz.
* SERVO: rotate to any given angle between 0–180 degrees.
  Example: "turn the servo to 90 degrees" → {{"device":"SERVO","angle":90}}.

Current LED state: {led_state}

Reply naturally but include a final JSON block of actions.
Example:
[
{{"device":"LED","action":"on"}},
{{"device":"SERVO","angle":45}}
]
"""
    reply = query_ollama(prompt)
    print("🤖 GPT-OSS reply:\n", reply)

    try:
        actions = re.findall(r'\{.*?\}', reply, flags=re.DOTALL)
        if not actions:
            print("⚠️ No JSON found — defaulting to LED status check.")
            request_led_status()
            return

        for act in actions:
            obj = json.loads(act.replace("'", "\""))
            device = obj.get("device", "").lower()

            if device == "led":
                action = obj.get("action", "").lower()
                if action == "on":
                    send_led_command("on")
                elif action == "off":
                    send_led_command("off")
                else:
                    request_led_status()

            elif device == "buzzer":
                freq = float(obj.get("frequency", 2000))
                dur = float(obj.get("duration", 0.5))
                send_buzzer_command(freq, dur)

            elif device == "servo":
                angle = float(obj.get("angle", 90))
                send_servo_command(angle)

            else:
                print(f"⚠️ Unknown device '{device}', ignoring.")

    except Exception as e:
        print("⚠️ Failed to parse LLM response:", e)

# ==========================
# DEEPGRAM AUDIO HANDLER
# ==========================

async def deepgram_main():
    global is_listening_for_command, command_text

    async with websockets.connect(
        DG_URL,
        extra_headers={"Authorization": f"Token {DEEPGRAM_API_KEY}"}
    ) as ws:
        print("✅ Connected to Deepgram. Say 'Hey Robert' to start...")

        loop = asyncio.get_running_loop()

        def callback(indata, frames, time, status):
            if status:
                print("⚠️", status)
            audio_data = (indata * 32767).astype(np.int16).tobytes()
            asyncio.run_coroutine_threadsafe(ws.send(audio_data), loop)

        async def send_audio():
            with sd.InputStream(callback=callback, channels=1, samplerate=16000, dtype="float32"):
                while True:
                    await asyncio.sleep(0.1)

        async def receive_transcripts():
            global is_listening_for_command, command_text
            last_speech_time = None

            async for message in ws:
                data = json.loads(message)
                if "channel" not in data:
                    continue

                alt = data["channel"]["alternatives"][0]
                transcript = alt.get("transcript", "").lower().strip()
                is_final = data.get("is_final", False)

                if not transcript:
                    continue

                print(f"🎙️ Transcript: {transcript}")

                if not is_listening_for_command and re.search(r"\bhey[, ]*robert\b", transcript):
                    is_listening_for_command = True
                    command_text = ""
                    print("🟢 Wake word detected! Listening for command...")
                    await send_buzzer_beep()
                    last_speech_time = asyncio.get_event_loop().time()
                    continue

                if is_listening_for_command:
                    command_text += " " + transcript
                    last_speech_time = asyncio.get_event_loop().time()

                    if is_final:
                        await asyncio.sleep(GRACE_DELAY)

                        if asyncio.get_event_loop().time() - last_speech_time >= GRACE_DELAY:
                            cleaned = command_text.strip()
                            if cleaned:
                                print(f"🗣️ Command heard: {cleaned}")
                                handle_command(cleaned)

                            is_listening_for_command = False
                            command_text = ""
                            print("\n🎤 Say 'Hey Robert' to wake me again...")

        await asyncio.gather(send_audio(), receive_transcripts())

# ==========================
# MAIN
# ==========================

try:
    asyncio.run(deepgram_main())
except KeyboardInterrupt:
    print("\n🛑 Stopped by user.")
