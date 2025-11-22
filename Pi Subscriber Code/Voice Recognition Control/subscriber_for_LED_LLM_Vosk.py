#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import RPi.GPIO as GPIO

# ==========================
# GPIO Setup
# ==========================
LED_PIN = 3
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

# ==========================
# MQTT Setup
# ==========================
MQTT_BROKER = "192.168.0.141"
CONTROL_TOPIC = "robot/control"
STATUS_TOPIC = "robot/status"
REQUEST_TOPIC = "robot/request"

led_state = "off"

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

    if msg.topic == CONTROL_TOPIC and payload.get("device") == "LED":
        if payload.get("state") == "on":
            GPIO.output(LED_PIN, GPIO.HIGH)
            led_state = "on"
            print("💡 LED turned ON")
        elif payload.get("state") == "off":
            GPIO.output(LED_PIN, GPIO.LOW)
            led_state = "off"
            print("🌑 LED turned OFF")
        publish_status(client)

    elif msg.topic == REQUEST_TOPIC and payload.get("device") == "LED" and payload.get("request") == "status":
        print("📨 Status requested by PC.")
        publish_status(client)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, 1883, 60)
client.loop_forever()
