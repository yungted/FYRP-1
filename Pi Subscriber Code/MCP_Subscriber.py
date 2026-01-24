#!/usr/bin/env python3
"""
Pi-side MCP-compatible device host for LLM executor.
Merged version:
- Keeps startup beep from version 1
- Integrates improved MCP action handler from version 2
"""

import paho.mqtt.client as mqtt
import json
import RPi.GPIO as GPIO
import time
import os

# =========================
# GPIO SETUP
# =========================
LED_PIN = 17        
BUZZER_PIN = 4
SERVO_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)
GPIO.output(BUZZER_PIN, GPIO.LOW)

# =========================
# STARTUP BEEP
# =========================
def startup_beep():
    print("🔊 Startup beep...")
    pwm = GPIO.PWM(BUZZER_PIN, 1000)
    pwm.start(50)
    time.sleep(0.15)
    pwm.stop()
    GPIO.output(BUZZER_PIN, GPIO.LOW)

startup_beep()

# =========================
# SERVO PWM INIT
# =========================
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
        {"name":"LED","id":"led","type":"digital_output","pin":LED_PIN,
         "actions":{"on":{},"off":{},"blink":{"parameters":{
             "times":{"type":"integer","default":3},
             "delay":{"type":"float","default":0.3}}}}},
        {"name":"Buzzer","id":"buzzer","type":"buzzer_output","pin":BUZZER_PIN,
         "actions":{"beep":{"parameters":{
             "frequency":{"type":"float","default":2000},
             "duration":{"type":"float","default":0.2}}},
             "sequence":{"parameters":{
                 "tone_sequence":{"type":"list"}}}}},
        {"name":"Servo Motor","id":"servo","type":"servo","pin":SERVO_PIN,
         "actions":{"rotate":{"parameters":{
             "angle":{"type":"float","min":0,"max":180}}}}}
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
# STATUS + MQTT HANDLERS
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
# INTEGRATED HANDLE_ACTION (patched for frequency notes)
# =========================
def handle_action(client, action_obj):
    if not isinstance(action_obj, dict):
        return

    mcp_action = str(action_obj.get("mcp_action", "")).lower()
    params = action_obj.get("parameters", [])
    if not isinstance(params, list):
        params = [params]

    for p in params:
        dev = (p.get("device") or "").lower()
        if not dev:
            continue

        if mcp_action == "control_device":
            # LED
            if dev == "led":
                state = (p.get("state") or "").lower()
                if state == "on":
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    STATE["led"] = "on"
                elif state == "off":
                    GPIO.output(LED_PIN, GPIO.LOW)
                    STATE["led"] = "off"
                elif state == "blink":
                    blink_led(int(p.get("times", 3)), float(p.get("delay", 0.3)))

            # BUZZER
            elif dev == "buzzer":

                # === ✔ PATCHED: frequency + duration always allowed ===
                if "frequency" in p or "duration" in p:
                    play_buzzer(
                        float(p.get("frequency", 2000)),
                        float(p.get("duration", 0.2))
                    )

                # Legacy beep action
                elif p.get("action") == "beep" or p.get("state") == "beep":
                    play_buzzer(
                        float(p.get("frequency", 2000)),
                        float(p.get("duration", 0.2))
                    )

                # Tone sequence (unchanged)
                elif "tone_sequence" in p:
                    for tone in p["tone_sequence"]:
                        play_buzzer(
                            float(tone.get("frequency_hz", 2000)),
                            float(tone.get("duration_s", 0.2))
                        )
                        time.sleep(0.05)

            # SERVO
            elif dev == "servo":
                set_servo_angle(float(p.get("angle", STATE["servo"])))

        # Delay support
        elif mcp_action == "delay":
            duration = float(p.get("duration_seconds", 0))
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

    if msg.topic == TOPIC_REQUEST:
        req = data.get("request")
        if req == "status":
            publish_status(client)
        elif req == "devices":
            client.publish(TOPIC_REGISTRY, json.dumps(DEVICE_REGISTRY))
        return

    if msg.topic == TOPIC_CONTROL:
        if isinstance(data, list):
            for action in data:
                handle_action(client, action)
        elif isinstance(data, dict):
            if "mcp_action" in data:
                handle_action(client, data)
            elif "device" in data:
                handle_action(client, {
                    "mcp_action": "control_device",
                    "parameters": [data]
                })

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