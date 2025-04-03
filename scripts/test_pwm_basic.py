import RPi.GPIO as GPIO
import time

# Define the pins connected to the stepper motor
stepPin = 3  # GPIO 3 (Pin 5 on Raspberry Pi)
dirPin = 4   # GPIO 4 (Pin 7 on Raspberry Pi)

# Define the delay between steps (adjust for speed)
lowStepDelay = 80e-6    # 80 microseconds
hiStepDelay = 4000e-6   # 4000 microseconds

stepDelay = 240e-6  # Initial delay in seconds for step interval

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(stepPin, GPIO.OUT)
GPIO.setup(dirPin, GPIO.OUT)

# Set direction
GPIO.output(dirPin, GPIO.HIGH)

try:
    while True:
        # Step the motor in a full-step sequence (forward direction)

        # Uncomment to gradually slow down the step speed
        # stepDelay *= 1.005
        # if stepDelay >= hiStepDelay:
        #     stepDelay = lowStepDelay

        for _ in range(3):
            # Step 1: Energize Phase 1
            GPIO.output(stepPin, GPIO.HIGH)
            time.sleep(stepDelay)

            # Step 2: Energize Phase 2
            GPIO.output(stepPin, GPIO.LOW)
            time.sleep(stepDelay)

except KeyboardInterrupt:
    print("Stopping motor...")
finally:
    GPIO.cleanup()
