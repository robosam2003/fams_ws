#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Location, WorkstationCommand
#from workstation.workstation import Location
from std_msgs.msg import String
#from machine_vision import eStop

class Workstation(Node):
    def __init__(self):
        super().__init__('workstation')
        self.get_logger().info('Workstation node has been started')

        self.workstation_id = 0
        self.available_operations = []
        self.status = 'FREE'  # Add a 'status' attribute with default value 'FREE'
        
        self.subscription = self.create_subscription(Location, 'location_topic', self.callback, 10)
        self.x = 0.0
        self.y = 0.0
        self.workstationCommandSubscription=self.create_subscription(String,'WorkstationCommand',self.cmdcallback,10 )

        #know how to determine if state= free/busy
        self.workstation_publisher = self.create_publisher(
            SystemState,
            'system_state',
            10
        )

        # SystemState1=SystemState
        # # Workstations State
        # workstation1.state='busy'

        # # Part State
        #     workstation1.parts

        # # Mobile Fleet State
        #     workstation1.mobile_robots
    def handle_part_input(self):
        workstation1= Workstation()
    
        workstation1.state='busy'
        #publish Workstation msg
    def handle_part_output(self):
        workstation1= Workstation()
    
        workstation1.state='free'
        
        self.workstation_publisher.publish("Updated State",workstation1.state)  # Replace with your actual workstation state

    
    def callback(self, msg):
        self.get_logger().info('Received location: X=%f, Y=%f' % (msg.x, msg.y))
    def cmdcallback(self, command:WorkstationCommand):
        self.get_logger().info('Received Command: ' )
        if command == "PART INPUT":
            self.handle_part_input()
        elif command == "PART OUTPUT":
            self.handle_part_output(self)
    

def main(args=None):
    rclpy.init(args=args)
    workstation_controller = Workstation()
    rclpy.spin(workstation_controller)
    workstation_controller.destroy_node()
    rclpy.shutdown()

    