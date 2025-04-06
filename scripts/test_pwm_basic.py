import RPi.GPIO as GPIO
import time
import math

# Define the pins connected to the stepper motor
stepPin = 3  # GPIO 3 (Pin 5 on Raspberry Pi)
dirPin = 4   # GPIO 4 (Pin 7 on Raspberry Pi)

# Define the delay range for the sinusoidal oscillation
lowStepDelay = 120e-6    # 280 microseconds
hiStepDelay = 5000e-6   # 4000 microseconds

# Set initial delay and time period for the oscillation
timePeriod = 6  # Period of the sinusoidal oscillation (in seconds)


# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(stepPin, GPIO.OUT)
GPIO.setup(dirPin, GPIO.OUT)

# Set initial direction
GPIO.output(dirPin, GPIO.HIGH)

try:
    start_time = time.time()  # Record the start time for the sinusoidal delay
    lastSwitch = time.time()

    while True:
        # Calculate the current time elapsed
        elapsed_time = time.time() - start_time

        # Sinusoidal variation for delay between steps (from lowStepDelay to hiStepDelay)
        delayFactor = (math.sin(2 * math.pi * elapsed_time / timePeriod) + 1) / 2
        stepDelay = lowStepDelay + delayFactor * (hiStepDelay - lowStepDelay)

        # Switch direction when the delay is at its largest (maximum delay)
        print(delayFactor, time.time() - lastSwitch)
        if delayFactor >= 0.98 and time.time() - lastSwitch > timePeriod//2:  # Close to maximum delay
            GPIO.output(dirPin, GPIO.HIGH if GPIO.input(dirPin) == GPIO.LOW else GPIO.LOW)
            lastSwitch = time.time()

        # Step the motor with the current delay
        GPIO.output(stepPin, GPIO.HIGH)
        time.sleep(stepDelay)

        GPIO.output(stepPin, GPIO.LOW)
        time.sleep(stepDelay)

except KeyboardInterrupt:
    print("Stopping motor...")
finally:
    GPIO.cleanup()
