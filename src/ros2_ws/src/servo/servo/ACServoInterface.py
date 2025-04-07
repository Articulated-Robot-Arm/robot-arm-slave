import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import time
import RPi.GPIO as GPIO

class ACServoInterface(Node):
    
    def __init__(self):
        super().__init__('ac_servo_interface')
        self.get_logger().info("Initializing AC Servo Interface Node...")

        # Unique ID for multi-motor scenarios (not used yet)
        self.uid = 1

        # Radians not yet used, but kept for future position limits
        self.minLimitRadians = 0
        self.maxLimitRadians = 0

        # Delay bounds in seconds (converted from microseconds)
        self.minLimitDelay = 120e-6
        self.maxLimitDelay = 4000e-6

        # GPIO pin setup (BCM numbering)
        self.stepPin = 3  # GPIO 3 (Pin 5 on Pi)
        self.dirPin = 4   # GPIO 4 (Pin 7 on Pi)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.stepPin, GPIO.OUT)
        GPIO.setup(self.dirPin, GPIO.OUT)

        self.get_logger().info("GPIO initialized. Subscribing to 'setVelocity'...")

        # Subscribe to velocity commands
        self.velocity_cmd_subscriber = self.create_subscription(
            Float32,
            'setVelocity',
            self.setPWMDelay,
            10
        )

    def setPWMDelay(self, msg: Float32):
        """
        Interprets delay and sends step pulses to motor.
        """
        delay = msg.data
        self.get_logger().info(f'Received delay: {delay:.3f}')

        # Clamp delay
        delay = max(min(delay, self.maxLimitDelay), self.minLimitDelay)

        # Set motor direction
        GPIO.output(self.dirPin, GPIO.HIGH if delay > 0 else GPIO.LOW)

        # Send step pulses (simple fixed-burst for now)
        for _ in range(5000):
            GPIO.output(self.stepPin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.stepPin, GPIO.LOW)
            time.sleep(delay)

    def destroy_node(self):
        self.get_logger().info("Cleaning up GPIO...")
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    acServoInterface = ACServoInterface()

    try:
        rclpy.spin(acServoInterface)
    except KeyboardInterrupt:
        acServoInterface.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        acServoInterface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
