#!/usr/bin/env python3
from std_msgs.msg import String
import rclpy

from rclpy.node import Node

from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Schedule
#from machine_vision import eStop, location
from workstation import Location
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
            self.system_state_handler,
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
    
    def publish_location(self):
        msg = Location()
        msg.x = 1.0  # Replace with your actual X coordinate
        msg.y = 2.0  # Replace with your actual Y coordinate
        self.publisher.publish(msg)
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

    
def main(args=None):
    rclpy.init(args=args)
    workstation_controller = WorkstationController()
    rclpy.spin(workstation_controller)
    workstation_controller.destroy_node()
    rclpy.shutdown()

    
    