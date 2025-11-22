#!/usr/bin/env python3
import argparse
import queue
import sys
import sounddevice as sd
import json
import RPi.GPIO as GPIO
from vosk import Model, KaldiRecognizer

q = queue.Queue()

def int_or_str(text):
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("-d", "--device", type=int_or_str, help="input device (numeric ID or substring)")
parser.add_argument("-r", "--samplerate", type=int, help="sampling rate")
args, remaining = parser.parse_known_args()

# -----------------------------
# Load Vosk model
# -----------------------------
model = Model("Models/vosk-model-small-en-us-0.15")

# Get sample rate if not specified
if args.samplerate is None:
    device_info = sd.query_devices(args.device, "input")
    args.samplerate = int(device_info["default_samplerate"])

# -----------------------------
# Setup GPIO
# -----------------------------
LED_PIN = 3
GPIO.setmode(GPIO.BCM)     # use Broadcom pin numbering
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

# -----------------------------
# Wake word setup
# -----------------------------
WAKE_WORD = "hey robert"
wake_active = False

# -----------------------------
# Start listening
# -----------------------------
with sd.RawInputStream(samplerate=args.samplerate, blocksize=8000,
                       device=args.device, dtype="int16",
                       channels=1, callback=callback):
    print("Listening... Say the wake word to activate.")

    rec = KaldiRecognizer(model, args.samplerate)

    try:
        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result["text"].lower()

                if not text:
                    continue

                if not wake_active:
                    if WAKE_WORD in text:
                        wake_active = True
                        print("Wake word detected! Listening for command...")
                else:
                    print("Command detected:", text)

                    # --- LED Commands ---
                    if "turn on the light" in text or "led on" in text:
                        GPIO.output(LED_PIN, GPIO.HIGH)
                        print("LED turned ON")

                    elif "turn off the light" in text or "led off" in text:
                        GPIO.output(LED_PIN, GPIO.LOW)
                        print("LED turned OFF")

                    # --- Exit Command ---
                    elif "exit" in text or "quit" in text:
                        print("Exiting...")
                        break

                    wake_active = False  # reset after one command

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        GPIO.cleanup()
        print("GPIO cleaned up. Program ended.")
