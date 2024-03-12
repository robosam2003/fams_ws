#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation
from workstation.workstation import Location

#from machine_vision import eStop

class Workstation(Node):
    def __init__(self):
        super().__init__('workstation_controller')
        self.get_logger().info('Workstation controller node has been started')

        self.workstation_id = 0
        self.available_operations = []
        self.status = 'FREE'  # Add a 'status' attribute with default value 'FREE'
        
        self.subscription = self.create_subscription(Location, 'location_topic', self.callback, 10)
        self.x = 0.0
        self.y = 0.0
    def callback(self, msg):
        self.get_logger().info('Received location: X=%f, Y=%f' % (msg.x, msg.y))

        


def main(args=None):
    rclpy.init(args=args)
    workstation_controller = Workstation()
    rclpy.spin(workstation_controller)
    workstation_controller.destroy_node()
    rclpy.shutdown()

    