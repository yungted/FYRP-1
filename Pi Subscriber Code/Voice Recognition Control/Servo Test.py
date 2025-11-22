import RPi.GPIO as GPIO
import time

# === CONFIG ===
SERVO_PIN = 17      # change if using a different GPIO pin
FREQ = 50           # servo PWM frequency (Hz)
MIN_DUTY = 2.5      # duty cycle for 0°
MAX_DUTY = 12.5     # duty cycle for 180°

# === SETUP ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, FREQ)
pwm.start(0)

def angle_to_duty(angle):
    """Convert 0–180° to duty cycle"""
    angle = max(0, min(180, angle))  # clamp range
    return MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (angle / 180.0)

def set_angle(angle):
    """Move servo to a specific angle"""
    duty = angle_to_duty(angle)
    pwm.ChangeDutyCycle(duty)
    print(f"→ Moving to {angle}°  (Duty {duty:.2f}%)")
    time.sleep(0.6)  # wait for servo to reach
    pwm.ChangeDutyCycle(0)  # stop sending signal

try:
    print("=== Servo Angle Input Test ===")
    print("Enter any angle between 0 and 180 (or 'q' to quit)\n")

    while True:
        user_input = input("Enter angle: ")
        if user_input.lower() in ['q', 'quit', 'exit']:
            break
        try:
            angle = float(user_input)
            set_angle(angle)
        except ValueError:
            print("⚠️ Please enter a valid number.")

finally:
    pwm.stop()
    GPIO.cleanup()
    print("\n✅ Servo test ended. GPIO cleaned up.")
