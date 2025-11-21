import os
import re
import io
import requests
import sounddevice as sd
import soundfile as sf
from ollama import Client

# =============================
# CONFIG
# =============================
DEEPGRAM_API_KEY = "dc36253a111632480faebd24920862ea1265df6a"
# USE ODYSSEUS VOICE
TTS_URL = "https://api.deepgram.com/v1/speak?model=aura-2-odysseus-en"

OLLAMA_KEY = "add508d2d17443e8b9953322d3751470.vmwVMnfKMep0zN42BPIMoidp"
os.environ["OLLAMA_API_KEY"] = OLLAMA_KEY

ollama_client = Client(
    host="https://ollama.com",
    headers={"Authorization": f"Bearer {OLLAMA_KEY}"}
)

# =============================
# CLOUD TTS (Deepgram Aura 2)
# =============================
def deepgram_tts(text):
    """Send text to Deepgram TTS and play WAV audio."""
    headers = {
        "Authorization": f"Token {DEEPGRAM_API_KEY}",
        "Accept": "audio/wav",
        "Content-Type": "application/json"
    }

    response = requests.post(TTS_URL, headers=headers, json={"text": text})

    if response.status_code != 200:
        print("❌ TTS Error:", response.text)
        return

    audio_bytes = response.content
    audio_buffer = io.BytesIO(audio_bytes)

    # Load WAV
    audio_np, samplerate = sf.read(audio_buffer)

    sd.play(audio_np, samplerate=samplerate)
    sd.wait()


# =============================
# CLOUD LLM
# =============================
def ask_ollama(prompt: str) -> str:
    system_message = "You are a helpful AI named Robert. Always reply concisely."

    try:
        res = ollama_client.chat(
            model="deepseek-v3.1:671b-cloud",
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": prompt}
            ],
            stream=False
        )
        text = res["message"]["content"]
        text = re.sub(r"```[\s\S]*?```", "", text)
        return text.strip()

    except Exception as e:
        print("⚠️ Ollama error:", e)
        return "Sorry, I couldn't generate a response."


# =============================
# MAIN LOOP
# =============================
if __name__ == "__main__":
    print("🤖 Chatbot ready! Type 'exit' to quit.")

    while True:
        msg = input("You: ")
        if msg.lower() == "exit":
            break

        reply = ask_ollama(msg)
        print("Bot:", reply)
        deepgram_tts(reply)
