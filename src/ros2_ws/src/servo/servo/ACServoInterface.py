import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import time
import threading
import RPi.GPIO as GPIO

from .Encoder import Encoder

class ACServoInterface(Node):

    def __init__(self):
        super().__init__('ac_servo_interface')
        self.get_logger().info("Initializing AC Servo Interface Node...")

        GPIO.cleanup()

        # Config
        self.minLimitDelay = 120e-6
        self.maxLimitDelay = 4000e-6
        self.stepPin = 3 # GPIO 3 = Pin 5
        self.dirPin = 4 # GPIO 4 = Pin 7
        self.run_duration_sec = 2.0

        self.encoderPinA = 17 # GPIO 17 = Pin 11
        self.encoderPinB = 27 # GPIO 27 = Pin 13

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.stepPin, GPIO.OUT)
        GPIO.setup(self.dirPin, GPIO.OUT)

        self.encoder = Encoder(self.encoderPinA, self.encoderPinB, self.get_logger())

        self.velocity_cmd_subscriber = self.create_subscription(
            Float32,
            'setVelocity',
            self.setPWMDelay,
            10
        )

        self.position_cmd_subscriber = self.create_subscription(
            Float32,
            'setPositionDegrees',
            self.setPositionDegrees,
            10
        )

        self.get_logger().info("setVelocity subscription setup")

    def setPositionDegrees(self, msg: Float32):
        goalAngle = msg.data
        self.get_logger().info(f"Received setPositionDegrees request for {goalAngle}")
        assert 0 <=  goalAngle, "Goal angle cannot be less than 0 degrees."
        assert 360 >= goalAngle, "Goal angle cannot be greater than 360 degrees."
        curAngle = self.encoder.get_angle()
        self.get_logger().info(f"CurAngle: {curAngle}, GoalAngle: {goalAngle}")

        minDelay = 180e-6
        maxDelay = 240e-4

        def getDelay(proportional):
            c = maxDelay - (proportional / 360.0) * (maxDelay - minDelay)
            if c < self.minLimitDelay:
                return self.minLimitDelay
            elif c > self.maxLimitDelay:
                return self.maxLimitDelay
            return c

        last_print = time.time()
        while curAngle - goalAngle > 1 or curAngle - goalAngle < -1:
            curAngle = self.encoder.get_angle()
            self.get_logger().info(f"CurAngle: {curAngle}, GoalAngle: {goalAngle}")
            for _ in range(5):
                delay = getDelay(abs(curAngle - goalAngle))
                GPIO.output(self.stepPin, GPIO.HIGH)
                time.sleep(delay)
                GPIO.output(self.stepPin, GPIO.LOW)
                time.sleep(delay)
                if time.time() - last_print > 0.5:
                    self.get_logger().info(f'Angle: {self.encoder.get_angle()}, delay = {delay}')
                    last_print = time.time()
        
        self.get_logger().info(f"FINAL: CurAngle: {curAngle}, GoalAngle: {goalAngle}")


    def setPWMDelay(self, msg: Float32):
        delay = msg.data
        self.get_logger().info(f'Received delay: {delay:.6f} s')

        delay = max(min(abs(delay), self.maxLimitDelay), self.minLimitDelay)
        direction = GPIO.HIGH if msg.data > 0 else GPIO.LOW
        GPIO.output(self.dirPin, direction)

        thread = threading.Thread(target=self._run_motor_fixed_time, args=(delay,))
        thread.daemon = True
        thread.start()

    def _run_motor_fixed_time(self, delay):
        self.get_logger().info(f"Running motor for {self.run_duration_sec}s with delay {delay:.6f}s")

        end_time = time.time() + self.run_duration_sec
        last_print = time.time()
        while time.time() < end_time:
            GPIO.output(self.stepPin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.stepPin, GPIO.LOW)
            time.sleep(delay)
            if time.time() - last_print > 0.5:
              self.get_logger().info(f'Angle: {self.encoder.get_angle()}')
              last_print = time.time()

        self.get_logger().info("Motor run complete.")

    def _log_encoder_status(self):
        ticks = self.encoder.get_ticks()
        angle = self.encoder.get_angle()
        self.get_logger().info(f"Encoder Ticks: {ticks} | Angle: {angle:.2f}°")

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