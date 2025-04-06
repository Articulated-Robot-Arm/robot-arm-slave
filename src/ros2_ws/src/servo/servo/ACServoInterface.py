import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ACServoInterface(Node):
    
    def __init__(self):
        super().__init__('ac_servo_interface')
        self.setup_subscriptions()
        self.uid = 1 # specifies node to control. Should be in the msgs
        self.minLimit = 0
        self.maxLimit = 0
        
    def setup_subscriptions(self):
        self.velocity_cmd_subscriber = self.create_subscription(
            Float32,
            'setVelocity',
            self.setVelocity, # callback
            10
        )
        self.velocity_cmd_subscriber # prevent unused variable warning

    def setServoPosition(self):
        pass
    
    def setVelocity(self):
        """
        Sets rotational rate percentage
        """
        self.get_logger().info('Received setVelocity command')
        pass

def main(args=None):
    rclpy.init(args=args)
    
    acServoInterface = ACServoInterface()
    
    rclpy.spin(acServoInterface)
    
    acServoInterface.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()