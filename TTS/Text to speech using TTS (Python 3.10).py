from TTS.api import TTS
import numpy as np
import sounddevice as sd

# Load VITS English model
tts = TTS("tts_models/en/ljspeech/vits")

def speak(text):
    audio = tts.tts(text=text)
    audio = np.array(audio, dtype=np.float32)
    sd.play(audio, samplerate=tts.synthesizer.output_sample_rate)
    sd.wait()

if __name__ == "__main__":
    while True:
        msg = input("Say something: ")
        if msg == "exit":
            break
        speak(msg)
