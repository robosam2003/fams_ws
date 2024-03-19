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
        

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

        self.system_state_subscriber = self.create_subscription(
            SystemState,
            'system_state',
            self.listener_callback,
            10
        )
        

        self.system_state_publisher = self.create_publisher(
            SystemState,
            'system_state',
            10
        )
        self.schedule_subscriber = self.create_subscription(
            Schedule,
            'schedule',
            self.callback_function,
            10
        )
        # # self.visionLocations_publisher = self.create_publisher(
        # #     visionLocations,
        # #     'visionLocations',
        # #     10
        # # )
        self.visionLocations_publisher = self.create_publisher(
            Location,
            'Location',
            10
        )
        self.publish_location
    def publish_location(self):
        msg = Location()
        msg.x = 1.0  # Replace with your actual X coordinate
        msg.y = 2.0  # Replace with your actual Y coordinate
        self.visionLocations_publisher.publish(msg)
        self.get_logger().info('Publishing location: X=%f, Y=%f' % (msg.x, msg.y))

        # self.eStop_subscriber = self.create_subscription(
        #     eStop,
        #     'eStop',
        #     10
        # )


        # Create several fake workstations and publish to state
        self.Workstation1 = Workstation()
        self.Workstation1.workstation_id = 1
        self.Workstation1.available_operations = ['32', '33']

        self.Workstation2 = Workstation()
        self.Workstation2.workstation_id = 2
        self.Workstation2.available_operations = ['22', '33']

        self.Workstation3 = Workstation()
        self.Workstation3.workstation_id = 3
        self.Workstation3.available_operations = ['22', '32']

        self.workstation_list = [self.Workstation1, self.Workstation2, self.Workstation3]
       
       
        w = Workstation()
        for w in self.workstation_list: w.status = 'FREE'
        self.system_state = SystemState()
        self.system_state.workstations = self.workstation_list
        self.system_state_publisher.publish(self.system_state)
        eStop=True
        if eStop==True:
            print("b is greater than a")
        #else
        # location x,y 
        # def timer_callback(self):
        #     msg = String()
        #     msg.data = 'Hello World: %d' % self.i
        #     self.system_state_publisher.publish(msg)
        #     self.get_logger().info('Publishing: "%s"' % msg.data)
        #     self.i += 1
    def listener_callback(self, msg):
            self.get_logger().info('I heard: ')
    def callback_function(self, msg):
        # Your callback logic goes here
        pass
def main(args=None):
    rclpy.init(args=args)
    workstation_controller = WorkstationController()
    rclpy.spin(workstation_controller)
    workstation_controller.destroy_node()
    rclpy.shutdown()

    
    