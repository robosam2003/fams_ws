#!/usr/bin/env python3
from std_msgs.msg import String
import rclpy

from rclpy.node import Node

from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Schedule, Location
#from machine_vision import eStop, location
#from workstation 
#from workstation.msg import Location
from rosidl_runtime_py import *
class WorkstationController(Node):
    def __init__(self):
        super().__init__('workstation_controller')
        self.get_logger().info('Workstation controller node has been started')
        
        # Subsciptions
        self.system_state_subscriber = self.create_subscription(
            SystemState,
            'system_state',
            self.system_state_handler,
            self.listener_callback,
            10
        )

        self.schedule_subscriber = self.create_subscription(
            Schedule,
            'schedule',
            self.schedule_handler,
            10
        )

        self.vision_locations_subscriber = self.create_subscription(
            Location, # What type of message will it be?
            'vision_location',
            self.vision_locations_handler,
            10
        )
        
        # Publishers    
        # self.system_state_publisher = self.create_publisher(
        #     SystemState,
        #     'system_state',
        #     10
        # )

        
    
    def publish_location(self):
        pass

    
def main(args=None):
    rclpy.init(args=args)
    workstation_controller = WorkstationController()
    rclpy.spin(workstation_controller)
    workstation_controller.destroy_node()
    rclpy.shutdown()

    
    