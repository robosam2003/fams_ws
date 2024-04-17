#!/usr/bin/env python3
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Schedule, Location
from rosidl_runtime_py import *


class WorkstationController(Node):
    def __init__(self):
        # This is a controller for the workstations in the system.
        # This will be run on a lab PC.
        super().__init__('workstation_controller')
        self.get_logger().info('Workstation controller node has been started')


        # Docking poses for the workstations
        w1_docking_pose = Pose()
        w1_docking_pose.position.x, w1_docking_pose.position.y, w1_docking_pose.position.z = 0, 0, 0
        w1_docking_pose.orientation.x, w1_docking_pose.orientation.y, w1_docking_pose.orientation.z, w1_docking_pose.orientation.w = 0, 0, 0, 0

        w2_docking_pose = Pose()
        w2_docking_pose.position.x, w2_docking_pose.position.y, w2_docking_pose.position.z = 0, 0, 0
        w2_docking_pose.orientation.x, w2_docking_pose.orientation.y, w2_docking_pose.orientation.z, w2_docking_pose.orientation.w = 0, 0, 0, 0
        self.workstation_docking_poses = [
            w1_docking_pose,
            w2_docking_pose
        ]

        # Operation time for the workstations depending on the operation
        self.workstation_operation_times = {
            'MILLING': 15,
            'DRILLING': 10,
            'LATHE': 20,
        }
        
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
        
        # Attributes that get updated
        self.system_state = SystemState()
        self.schedule = Schedule()
    
    def system_state_handler(self, msg):
        # Update local system state attribute
        self.system_state = msg
    
    def schedule_handler(self, msg):
        # Update local schedule attribute
        self.schedule = msg

        # Parse schedule into instructions
        # Extract parts, subprocesses, workstations - Instructions are matched by index
        parts = self.schedule.parts
        subprocesses = self.schedule.subprocesses
        workstations = self.schedule.workstations

        # Should we wait until the robot is in position? or should we have an eternal loop that check it, 
        #  and calls this function again? or another function that generates the workstationCommand?









    
def main(args=None):
    rclpy.init(args=args)
    workstation_controller = WorkstationController()
    rclpy.spin(workstation_controller)
    workstation_controller.destroy_node()
    rclpy.shutdown()

    
    