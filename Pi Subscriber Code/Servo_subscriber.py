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
SERVO_PIN = 17  

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

GPIO.output(LED_PIN, GPIO.LOW)
GPIO.output(BUZZER_PIN, GPIO.LOW)

# ==========================
# Servo Setup
# ==========================
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz = standard for servo
servo_pwm.start(0)
servo_angle = 90  # initial position

def set_servo_angle(angle):
    global servo_angle
    if angle < 0: angle = 0
    if angle > 180: angle = 180

    # Map 0–180° to 5–10% duty cycle (1–2 ms)
    duty = 5 + (angle / 180) * 5  
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)
    servo_angle = angle
    print(f"🧭 Servo moved to {angle}° (Duty = {duty:.2f}%)")

# Center servo at startup
set_servo_angle(servo_angle)

# ==========================
# MQTT Setup
# ==========================
MQTT_BROKER = "192.168.0.165"
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
        "device": "system_status",
        "LED": led_state,
        "Servo": servo_angle
    })
    client.publish(STATUS_TOPIC, msg)
    print(f"📡 Published system status → {msg}")

def on_connect(client, userdata, flags, rc):
    print(f"✅ Connected to MQTT broker ({MQTT_BROKER}) with code {rc}")
    client.subscribe(CONTROL_TOPIC)
    client.subscribe(REQUEST_TOPIC)
    publish_status(client)

def on_message(client, userdata, msg):
    global led_state, servo_angle
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
            play_buzzer()
        elif payload.get("action") == "buzz":
            freq = float(payload.get("frequency", 2000))
            dur = float(payload.get("duration", 0.5))
            play_buzzer(freq, dur)

    # --- Servo handling ---
    elif payload.get("device") == "SERVO":
        angle = payload.get("angle")
        if angle is not None:
            try:
                angle = float(angle)
                set_servo_angle(angle)
                publish_status(client)
            except ValueError:
                print("⚠️ Invalid angle value received.")

    # --- Status request ---
    elif payload.get("request") == "status":
        print("📨 Status requested by PC.")
        publish_status(client)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, 1883, 60)
client.loop_forever()
