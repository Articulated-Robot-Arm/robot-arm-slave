import RPi.GPIO as GPIO
import threading
import time

class Encoder:
    def __init__(self, pin_a, pin_b, ticks_per_rev=400, degrees_per_rev=360.0, poll_delay=0.001):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ticks_per_rev = ticks_per_rev
        self.degrees_per_rev = degrees_per_rev
        self.poll_delay = poll_delay  # Polling interval in seconds (1 ms default)

        # Setup GPIO
        GPIO.setup(pin_a, GPIO.IN)
        GPIO.setup(pin_b, GPIO.IN)

        # Encoder state
        self.encoder_ticks = 0
        self.prev_state_a = GPIO.input(pin_a)
        self.prev_state_b = GPIO.input(pin_b)

        self.lock = threading.Lock()

        # Start polling thread
        self._running = True
        self.poll_thread = threading.Thread(target=self._poll_encoder)
        self.poll_thread.daemon = True
        self.poll_thread.start()

    def _poll_encoder(self):
        while self._running:
            state_a = GPIO.input(self.pin_a)
            state_b = GPIO.input(self.pin_b)

            with self.lock:
                if self.prev_state_a == GPIO.LOW and state_a == GPIO.HIGH:  # Rising edge
                    if state_b == GPIO.LOW:
                        self.encoder_ticks += 1
                    else:
                        self.encoder_ticks -= 1
                elif self.prev_state_a == GPIO.HIGH and state_a == GPIO.LOW:  # Falling edge
                    if state_b == GPIO.HIGH:
                        self.encoder_ticks += 1
                    else:
                        self.encoder_ticks -= 1

                self.prev_state_a = state_a
                self.prev_state_b = state_b

            time.sleep(self.poll_delay)

    def get_ticks(self):
        with self.lock:
            return self.encoder_ticks

    def get_angle(self):
        with self.lock:
            return (self.encoder_ticks % self.ticks_per_rev) * self.degrees_per_rev / self.ticks_per_rev

    def stop(self):
        self._running = False
        self.poll_thread.join()