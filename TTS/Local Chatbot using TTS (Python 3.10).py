from TTS.api import TTS
from ollama import Client
import numpy as np
import sounddevice as sd
import os
import re

# ----------------------------
# TTS setup
# ----------------------------
tts = TTS("tts_models/en/ljspeech/vits")

def speak(text):
    audio = tts.tts(text=text)
    audio = np.array(audio, dtype=np.float32)
    sd.play(audio, samplerate=tts.synthesizer.output_sample_rate)
    sd.wait()

# ----------------------------
# Ollama setup
# ----------------------------
OLLAMA_KEY = "add508d2d17443e8b9953322d3751470.vmwVMnfKMep0zN42BPIMoidp"
os.environ["OLLAMA_API_KEY"] = OLLAMA_KEY
ollama_client = Client(host="https://ollama.com", headers={"Authorization": f"Bearer {OLLAMA_KEY}"})

def ask_ollama(prompt: str) -> str:
    try:
        system_message = "You are a helpful assistant. Always respond in short and concise sentences."
        res = ollama_client.chat(
            model="gpt-oss:120b-cloud",
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": prompt}
            ],
            stream=False
        )
        text = res["message"]["content"]
        text = re.sub(r"```[\s\S]*?```", "", text)  # remove code blocks
        return text.strip()
    except Exception as e:
        print("⚠️ Ollama error:", e)
        return "Sorry, I couldn't generate a response."


# ----------------------------
# Chat loop
# ----------------------------
if __name__ == "__main__":
    print("🤖 Chatbot ready! Type 'exit' to quit.")
    while True:
        msg = input("You: ")
        if msg.lower() == "exit":
            break
        response = ask_ollama(msg)
        print("Bot:", response)
        speak(response)
