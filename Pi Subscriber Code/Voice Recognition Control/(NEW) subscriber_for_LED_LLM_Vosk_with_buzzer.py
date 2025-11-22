#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import RPi.GPIO as GPIO
import time

# ==========================
# GPIO Setup
# ==========================
LED_PIN = 3
BUZZER_PIN = 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

GPIO.output(LED_PIN, GPIO.LOW)
GPIO.output(BUZZER_PIN, GPIO.LOW)

# ==========================
# MQTT Setup
# ==========================
MQTT_BROKER = "192.168.0.141"
CONTROL_TOPIC = "robot/control"
STATUS_TOPIC = "robot/status"
REQUEST_TOPIC = "robot/request"

led_state = "off"

def play_buzzer(frequency=2000, duration=0.5):
    """Play a tone at a given frequency and duration."""
    print(f"🔊 Playing buzzer at {frequency} Hz for {duration} s")
    pwm = GPIO.PWM(BUZZER_PIN, frequency)
    pwm.start(50)
    time.sleep(duration)
    pwm.stop()
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    print("✅ Buzzer done")

def publish_status(client):
    msg = json.dumps({
        "device": "LED",
        "pin": LED_PIN,
        "state": led_state
    })
    client.publish(STATUS_TOPIC, msg)
    print(f"📡 Published LED status → {msg}")

def on_connect(client, userdata, flags, rc):
    print(f"✅ Connected to MQTT broker ({MQTT_BROKER}) with code {rc}")
    client.subscribe(CONTROL_TOPIC)
    client.subscribe(REQUEST_TOPIC)
    publish_status(client)

def on_message(client, userdata, msg):
    global led_state
    payload = json.loads(msg.payload.decode())
    print(f"📥 Received on {msg.topic}: {payload}")

    # --- LED handling ---
    if payload.get("device") == "LED":
        if payload.get("state") == "on":
            GPIO.output(LED_PIN, GPIO.HIGH)
            led_state = "on"
            print("💡 LED turned ON")
        elif payload.get("state") == "off":
            GPIO.output(LED_PIN, GPIO.LOW)
            led_state = "off"
            print("🌑 LED turned OFF")
        publish_status(client)

    # --- Buzzer handling ---
    elif payload.get("device") == "BUZZER":
        if payload.get("action") == "beep":
            play_buzzer()  # default beep
        elif payload.get("action") == "buzz":
            freq = float(payload.get("frequency", 2000))
            dur = float(payload.get("duration", 0.5))
            play_buzzer(freq, dur)

    # --- Status request ---
    elif payload.get("device") == "LED" and payload.get("request") == "status":
        print("📨 Status requested by PC.")
        publish_status(client)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, 1883, 60)
client.loop_forever()
