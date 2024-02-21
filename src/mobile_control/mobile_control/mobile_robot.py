import rclpy
from rclpy.node import Node
import serial 
from fams_interfaces.msg import BasicControl

class MobileRobot(rclpy.node.Node):
    def __init__(self):
        super().__init__('mobile_robot')
        self.get_logger().info('Mobile robot node has been started')
        self.create_subscription(BasicControl, 
                                 'basic_control',
                                  self.basic_control_callback, 
                                  10)
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
    
    def basic_control_callback(self, msg):
        self.get_logger().info('Received basic control message: %s' % msg)
        # Publish the message over serial to the robot
        s = msg.direction.encode('utf-8')
        self.ser.write(s)
        self.get_logger().info('Sent basic control message: %s' % msg.direction)

def main(args=None):
    rclpy.init(args=args)
    mobile_robot = MobileRobot()
    rclpy.spin(mobile_robot)
    mobile_robot.destroy_node()
    rclpy.shutdown()    