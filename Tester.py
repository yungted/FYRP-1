import asyncio
import edge_tts

async def main():
    # Set the voice to RyanNeural
    voice = "en-US-RyanNeural"
    
    # Input text
    text = "Hello, this is Ryan speaking locally using Edge TTS."
    
    # Output file
    output_file = "ryan_local.mp3"
    
    # Create TTS object and save to file
    communicator = edge_tts.Communicate(text, voice)
    await communicator.save(output_file)
    print(f"Saved synthesized speech to {output_file}")

# Run the async function
asyncio.run(main())
