import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import time
import threading
import RPi.GPIO as GPIO

class ACServoInterface(Node):

    def __init__(self):
        super().__init__('ac_servo_interface')
        self.get_logger().info("Initializing AC Servo Interface Node...")

        # Motor config
        self.minLimitDelay = 120e-6
        self.maxLimitDelay = 4000e-6
        self.stepPin = 3
        self.dirPin = 4

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.stepPin, GPIO.OUT)
        GPIO.setup(self.dirPin, GPIO.OUT)

        # ROS
        self.velocity_cmd_subscriber = self.create_subscription(
            Float32,
            'setVelocity',
            self.setPWMDelay,
            10
        )

        # Thread-safe version control
        self.lock = threading.Lock()
        self.command_version = 0

    def setPWMDelay(self, msg: Float32):
        delay = msg.data
        self.get_logger().info(f'Received delay: {delay:.6f} s')

        delay = max(min(abs(delay), self.maxLimitDelay), self.minLimitDelay)
        direction = GPIO.HIGH if msg.data > 0 else GPIO.LOW
        GPIO.output(self.dirPin, direction)

        with self.lock:
            self.command_version += 1
            my_version = self.command_version

        thread = threading.Thread(target=self._run_motor_continuous, args=(delay, my_version))
        thread.daemon = True
        thread.start()

    def _run_motor_continuous(self, delay, version):
        self.get_logger().info(f"Starting motor loop (delay={delay:.6f}s)")

        while True:
            with self.lock:
                if version != self.command_version:
                    self.get_logger().info(f"Stopping motor thread (version {version})")
                    return

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
    node = ACServoInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
