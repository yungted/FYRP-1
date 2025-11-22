import RPi.GPIO as GPIO
import time

# --- Setup ---
BUZZER_PIN = 4  # Passive buzzer connected to GPIO 4
FREQUENCY1 = 1700  # Hz - first tone
FREQUENCY2 = 2000  # Hz - second tone
DURATION = 0.2      # seconds per tone

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Create one PWM instance
pwm = GPIO.PWM(BUZZER_PIN, FREQUENCY1)

try:
    print(f"🔔 Playing first note at {FREQUENCY1} Hz")
    pwm.start(50)  # Start PWM at 50% duty cycle
    time.sleep(DURATION)

    print(f"🎵 Switching to second note at {FREQUENCY2} Hz")
    pwm.ChangeFrequency(FREQUENCY2)
    time.sleep(DURATION)

    pwm.stop()
    print("✅ Done — you should have heard two distinct tones!")

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
