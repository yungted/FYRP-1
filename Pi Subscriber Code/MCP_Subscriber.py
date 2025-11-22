#!/usr/bin/env python3
"""
Pi-side script:
- Publishes a retained registry message on connect (robot/registry) describing available inputs/outputs.
- Subscribes to robot/control to receive MCP JSON control messages from PC.
- Subscribes to robot/request to handle simple requests (e.g., {"request":"status"}).
- Executes simple hardware actions: LED, buzzer, servo (GPIO).
- Publishes status updates to robot/status.

Prereqs on Pi:
sudo apt install python3-pip
pip3 install paho-mqtt
Also ensure RPi.GPIO is available (on Raspbian / Raspberry Pi OS).
"""

import paho.mqtt.client as mqtt
import json
import RPi.GPIO as GPIO
import time
import os

# ==========================
# GPIO Setup (edit pins as your wiring requires)
# ==========================
LED_PIN = 3       # example BCM pin for LED
BUZZER_PIN = 4    # example BCM pin for buzzer (use PWM for tone)
SERVO_PIN = 17    # example BCM pin for servo (PWM)

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)
GPIO.output(BUZZER_PIN, GPIO.LOW)

# servo PWM (50Hz typical)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)

# Global simple state
led_state = "off"
servo_angle = 90

# ==========================
# MQTT Configuration
# ==========================
MQTT_BROKER = os.environ.get("MQTT_BROKER", "192.168.0.165")  # PC broker IP if running broker on PC
REG_TOPIC = "robot/registry"
CONTROL_TOPIC = "robot/control"
STATUS_TOPIC = "robot/status"
REQUEST_TOPIC = "robot/request"

CLIENT_ID = "pi-01"

# ==========================
# Device registry (what the Pi offers) — published retained on connect
# Edit this to reflect real hardware
# ==========================
DEVICE_REGISTRY = {
    "device_id": CLIENT_ID,
    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "inputs": [
        {"name": "button1", "type": "digital", "pin": 17, "description": "user button A"},
        {"name": "distance", "type": "sensor", "unit": "cm", "description": "ultrasonic sensor"}
    ],
    "outputs": [
        {"name": "led", "type": "digital", "pin": LED_PIN, "actions": ["on", "off", "blink"]},
        {"name": "buzzer", "type": "buzzer", "pin": BUZZER_PIN, "actions": ["beep", "sequence"]},
        {"name": "servo1", "type": "servo", "pin": SERVO_PIN, "angle_range": [0, 180]}
    ]
}

# ==========================
# Hardware control helper functions
# ==========================
def set_servo_angle(angle):
    """Rotate servo to the given angle (0–180)."""
    global servo_angle
    angle = max(0, min(180, float(angle)))
    # Convert angle to duty cycle roughly between 2.5 and 12.5 (adjust for your servo)
    duty = 2.5 + (angle / 180.0) * 10.0
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.4)            # wait a bit for servo to move
    servo_pwm.ChangeDutyCycle(0)
    servo_angle = angle
    print(f"🧭 Servo → {angle}°")

def play_buzzer(freq=2000, duration=0.5):
    """Play a tone using PWM on the buzzer pin. Not all buzzers support tone generation."""
    pwm = GPIO.PWM(BUZZER_PIN, freq)
    pwm.start(50)   # 50% duty
    time.sleep(duration)
    pwm.stop()
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    print(f"🎵 Buzzer: {freq} Hz for {duration} s")

def blink_led(times=3, delay=0.3):
    """Blink LED on and off a number of times."""
    global led_state
    for _ in range(times):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(delay)
    led_state = "off"
    print("💡 LED blinked")

# ==========================
# MQTT callbacks
# ==========================
def on_connect(client, userdata, flags, rc):
    """Called when the Pi connects to MQTT broker."""
    print("✅ Connected to MQTT broker:", MQTT_BROKER, "rc=", rc)
    # Publish the device registry as retained so the MCP server / PC receives it immediately
    try:
        client.publish(REG_TOPIC, json.dumps(DEVICE_REGISTRY), retain=True)
        print("📦 Published device registry (retained) to", REG_TOPIC)
    except Exception as e:
        print("⚠️ Failed publishing registry:", e)
    # Subscribe to control and request topics
    client.subscribe(CONTROL_TOPIC)
    client.subscribe(REQUEST_TOPIC)
    print("Subscribed to topics:", CONTROL_TOPIC, REQUEST_TOPIC)
    # Also send an initial status
    publish_status(client)

def on_message(client, userdata, msg):
    """Handle control messages and requests."""
    global led_state
    try:
        payload = json.loads(msg.payload.decode())
    except Exception as e:
        print("⚠ Invalid payload:", e)
        return

    print(f"📥 Received on {msg.topic}: {json.dumps(payload)}")

    # Control path: expected MCP JSON with "parameters" list
    if msg.topic == CONTROL_TOPIC:
        params = payload.get("parameters") or []
        # Allow root-level 'device' for simple commands as well
        if not params and (payload.get("device") or payload.get("action")):
            params = [payload]

        for p in params:
            dev = (p.get("device") or "").upper()

            # ===== LED =====
            if dev == "LED":
                state = (p.get("state") or p.get("action") or "").lower()
                if state == "on":
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    led_state = "on"
                    print("💡 LED ON")
                elif state == "off":
                    GPIO.output(LED_PIN, GPIO.LOW)
                    led_state = "off"
                    print("🌑 LED OFF")
                elif state == "blink":
                    times = int(p.get("times", 3))
                    delay = float(p.get("delay", 0.3))
                    blink_led(times=times, delay=delay)
                publish_status(client)

            # ===== BUZZER =====
            elif dev == "BUZZER":
                # Support a tone sequence if provided
                if "tone_sequence" in p:
                    tones = p["tone_sequence"]
                    print(f"🎶 Playing sequence of {len(tones)} tones...")
                    for tone in tones:
                        freq = float(tone.get("frequency_hz", tone.get("freq", 2000)))
                        dur = float(tone.get("duration_s", tone.get("duration", 0.5)))
                        play_buzzer(freq, dur)
                        time.sleep(0.05)
                else:
                    action = (p.get("action") or "").lower()
                    if action == "beep":
                        play_buzzer(2000, 0.2)
                    elif action == "buzz":
                        freq = float(p.get("frequency", 2000))
                        dur = float(p.get("duration", 0.5))
                        play_buzzer(freq, dur)

            # ===== SERVO =====
            elif dev == "SERVO":
                angle = p.get("angle")
                if angle is None:
                    # support nested parameters
                    angle = p.get("parameters", {}).get("angle", 90)
                try:
                    set_servo_angle(float(angle))
                except Exception as e:
                    print("⚠ Invalid servo angle:", e)
                publish_status(client)

            else:
                print("⚠ Unknown/unsupported device command:", dev)

    # Request path (e.g., {"request": "status"})
    elif msg.topic == REQUEST_TOPIC:
        req = payload.get("request")
        if req == "status":
            print("📨 Status requested by remote.")
            publish_status(client)

def publish_status(client):
    """Send a small status object back to the PC to indicate current device states."""
    status = {
        "device": "system_status",
        "device_id": CLIENT_ID,
        "LED": led_state,
        "Servo": servo_angle,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    }
    try:
        client.publish(STATUS_TOPIC, json.dumps(status))
        print(f"📡 Published status: {status}")
    except Exception as e:
        print("⚠️ Failed to publish status:", e)

# ==========================
# MAIN MQTT LOOP
# ==========================
def main():
    # Compatible with paho-mqtt v2.x (uses VERSION1 for legacy callbacks)
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1, client_id=CLIENT_ID)

    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to broker (replace with your PC's broker IP if broker is there)
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
