import rclpy
from rclpy.node import Node

from fams_interfaces.msg import BasicControl

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.get_logger().info('Keyboard controller node has been started')
        self.publisher = self.create_publisher(BasicControl, 'basic_control', 10)
        
        while True:
            direction = input('Enter a direction: ')
            msg = BasicControl()
            msg.direction = direction
            self.get_logger().info('Publishing basic control message: %s' % msg.direction)
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()


        

