#!/usr/bin/env python3

import argparse
import queue
import sys
import json
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import paho.mqtt.client as mqtt
import time

# -------------------- MQTT CONFIGURATION --------------------
BROKER_IP = "192.168.0.165"
PORT = 1883
TOPIC = "speech/recognized"

# Create MQTT client and connect
client = mqtt.Client()
try:
    client.connect(BROKER_IP, PORT, 60)
    client.loop_start()
    print(f"Connected to MQTT broker at {BROKER_IP}:{PORT}")
except Exception as e:
    print(f"MQTT connection failed: {e}")

# -------------------- AUDIO STREAM CONFIGURATION --------------------
q = queue.Queue()

def int_or_str(text):
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time_info, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

# -------------------- ARGUMENT PARSING --------------------
parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("-l", "--list-devices", action="store_true", help="show list of audio devices and exit")
args, remaining = parser.parse_known_args()
if args.list_devices:
    print(sd.query_devices())
    parser.exit(0)

parser = argparse.ArgumentParser(parents=[parser])
parser.add_argument("-f", "--filename", type=str, metavar="FILENAME", help="audio file to store recording to")
parser.add_argument("-d", "--device", type=int_or_str, help="input device (numeric ID or substring)")
parser.add_argument("-r", "--samplerate", type=int, help="sampling rate")
parser.add_argument("-m", "--model", type=str, help="language model; e.g. en-us, fr, nl; default is en-us")
args = parser.parse_args(remaining)

print("Listening... (say something)")

# -------------------- MAIN RECOGNITION LOOP --------------------
try:
    if args.samplerate is None:
        device_info = sd.query_devices(args.device, "input")
        args.samplerate = int(device_info["default_samplerate"])

    if args.model is None:
        model = Model("Models/vosk-model-small-en-us-0.15")
    else:
        model = Model(args.model)

    dump_fn = open(args.filename, "wb") if args.filename else None

    with sd.RawInputStream(samplerate=args.samplerate, blocksize=8000,
                           device=args.device, dtype="int16", channels=1, callback=callback):
        rec = KaldiRecognizer(model, args.samplerate)

        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get("text", "")
                if text:
                    print(f"You said: {text}")
                    # ✅ Publish to MQTT
                    client.publish(TOPIC, text)
                    print(f"Sent to MQTT → {TOPIC}: {text}")
                    # Stop if user says "exit"
                    if text.strip().lower() == "exit":
                        break
            if dump_fn:
                dump_fn.write(data)

except KeyboardInterrupt:
    print("\nStopped by user")

except Exception as e:
    print(f"Error: {e}")

finally:
    client.loop_stop()
    client.disconnect()
    print("Disconnected from MQTT broker")
    if dump_fn:
        dump_fn.close()
