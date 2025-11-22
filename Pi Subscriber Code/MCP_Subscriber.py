#!/usr/bin/env python3
"""
Pi-side MCP-compatible device host for LLM executor.

Features:
- Publishes MCP device registry (retained) on MQTT.
- Subscribes to control and request topics.
- Accepts MCP-style multi-step actions and legacy simple actions.
- Publishes status updates after each action.
"""

import paho.mqtt.client as mqtt
import json
import RPi.GPIO as GPIO
import time
import os

# =========================
# GPIO SETUP
# =========================
LED_PIN = 3
BUZZER_PIN = 4
SERVO_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)
GPIO.output(BUZZER_PIN, GPIO.LOW)

servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)

# =========================
# GLOBAL STATE
# =========================
STATE = {
    "led": "off",
    "buzzer": "idle",
    "servo": 90
}

# =========================
# MQTT CONFIG
# =========================
MQTT_BROKER = os.environ.get("MQTT_BROKER", "192.168.0.165")
TOPIC_REGISTRY = "robot/device_registry"
TOPIC_CONTROL  = "robot/control"
TOPIC_STATUS   = "robot/status"
TOPIC_REQUEST  = "robot/request"

CLIENT_ID = "pi-01"

# =========================
# DEVICE REGISTRY
# =========================
DEVICE_REGISTRY = {
    "device_id": CLIENT_ID,
    "device_type": "raspberry_pi",
    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "devices": [
        {"name":"LED","id":"led","type":"digital_output","pin":LED_PIN,"actions":{"on":{},"off":{},"blink":{"parameters":{"times":{"type":"integer","default":3},"delay":{"type":"float","default":0.3}}}}},
        {"name":"Buzzer","id":"buzzer","type":"buzzer_output","pin":BUZZER_PIN,"actions":{"beep":{"parameters":{"frequency":{"type":"float","default":2000},"duration":{"type":"float","default":0.2}}},"sequence":{"parameters":{"tone_sequence":{"type":"list"}}}}},
        {"name":"Servo Motor","id":"servo","type":"servo","pin":SERVO_PIN,"actions":{"rotate":{"parameters":{"angle":{"type":"float","min":0,"max":180}}}}}
    ]
}

# =========================
# HARDWARE CONTROL FUNCTIONS
# =========================
def set_servo_angle(angle):
    angle = max(0, min(180, float(angle)))
    duty = 2.5 + (angle / 180.0) * 10.0
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.4)
    servo_pwm.ChangeDutyCycle(0)
    STATE["servo"] = angle
    print(f"🧭 Servo → {angle}°")

def play_buzzer(freq=2000, duration=0.2):
    pwm = GPIO.PWM(BUZZER_PIN, freq)
    pwm.start(50)
    time.sleep(duration)
    pwm.stop()
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    STATE["buzzer"] = "idle"
    print(f"🎵 Buzzer {freq}Hz for {duration}s")

def blink_led(times=3, delay=0.3):
    for _ in range(times):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(delay)
    STATE["led"] = "off"
    print("💡 LED blinked")

# =========================
# STATUS PUBLISHER
# =========================
def publish_status(client):
    status = {
        "device_id": CLIENT_ID,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "state": STATE
    }
    client.publish(TOPIC_STATUS, json.dumps(status))
    print("📡 Published status:", status)

# =========================
# UTILITIES
# =========================
def find_device_by_id(dev_id):
    dev_id = str(dev_id).lower()
    for d in DEVICE_REGISTRY["devices"]:
        if d["id"].lower() == dev_id:
            return d
    return None

# =========================
# ACTION HANDLER
# =========================
def handle_action(client, action_obj):
    if not isinstance(action_obj, dict):
        return
    mcp_action = str(action_obj.get("mcp_action","")).lower()
    params = action_obj.get("parameters", [])
    if not isinstance(params, list):
        params = [params]

    for p in params:
        dev = (p.get("device") or "").lower()
        if not dev:
            continue

        # Control Device
        if mcp_action == "control_device":
            if dev == "led":
                state = (p.get("state") or "").lower()
                if state == "on":
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    STATE["led"] = "on"
                elif state == "off":
                    GPIO.output(LED_PIN, GPIO.LOW)
                    STATE["led"] = "off"
                elif state == "blink":
                    blink_led(int(p.get("times",3)), float(p.get("delay",0.3)))
            elif dev == "buzzer":
                if p.get("action")=="beep" or p.get("state")=="beep":
                    play_buzzer(float(p.get("frequency",2000)), float(p.get("duration",0.2)))
                elif "tone_sequence" in p:
                    for tone in p["tone_sequence"]:
                        play_buzzer(float(tone.get("frequency_hz",2000)), float(tone.get("duration_s",0.2)))
                        time.sleep(0.05)
            elif dev == "servo":
                set_servo_angle(float(p.get("angle",STATE["servo"])))

        # Delay action
        elif mcp_action == "delay":
            duration = float(p.get("duration_seconds",0))
            print(f"⏳ Delay {duration}s")
            time.sleep(duration)

    publish_status(client)

# =========================
# ON MESSAGE
# =========================
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
    except:
        print("⚠ Invalid JSON")
        return
    print(f"📥 Received on {msg.topic}: {data}")

    # Requests (optional)
    if msg.topic == TOPIC_REQUEST:
        req = data.get("request")
        if req == "status":
            publish_status(client)
        elif req == "devices":
            client.publish(TOPIC_REGISTRY, json.dumps(DEVICE_REGISTRY))
            print("📨 Device list requested")
        return

    # Control
    if msg.topic == TOPIC_CONTROL:
        # Accept MCP-style list of actions
        if isinstance(data, list):
            for action in data:
                handle_action(client, action)
        elif isinstance(data, dict):
            # Single MCP action or legacy simple action
            if "mcp_action" in data:
                handle_action(client, data)
            elif "device" in data:
                # coerce legacy simple control to MCP style
                handle_action(client, {"mcp_action":"control_device","parameters":[data]})
        else:
            print("⚠ Unknown control format")

# =========================
# ON CONNECT
# =========================
def on_connect(client, userdata, flags, rc):
    print(f"✅ Connected to MQTT {MQTT_BROKER} rc={rc}")
    client.publish(TOPIC_REGISTRY, json.dumps(DEVICE_REGISTRY), retain=True)
    client.subscribe(TOPIC_CONTROL)
    client.subscribe(TOPIC_REQUEST)
    publish_status(client)

# =========================
# MAIN
# =========================
def main():
    client = mqtt.Client(client_id=CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, 1883, 60)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("🛑 Shutting down...")
    finally:
        servo_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
