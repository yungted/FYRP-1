# test_tts.py
from TTS.api import TTS
import os

# --- Initialize TTS ---
# This will download a default English model if not already downloaded
tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=True, gpu=False)

# --- Text to Convert ---
text = "Hello Rodney! This is a test of your local text-to-speech system using Python."

# --- Output File ---
output_file = "output.wav"

# --- Generate Speech ---
tts.tts_to_file(text=text, file_path=output_file)
print(f"✅ Speech saved to {output_file}")

# --- Play Audio ---
try:
    if os.name == "nt":  # Windows
        os.system(f'start {output_file}')
    else:
        os.system(f'afplay {output_file}')  # macOS
except Exception as e:
    print(f"⚠️ Could not play audio automatically: {e}")
