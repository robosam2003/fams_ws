#!/usr/bin/env python3
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Schedule, Location, Vision, WorkstationCommand
from rosidl_runtime_py import *
import math
import numpy as np


class WorkstationController(Node):
    def __init__(self):
        # This is a controller for the workstations in the system.
        # This will be run on a lab PC.
        super().__init__('workstation_controller')
        self.get_logger().info('Workstation controller node has been started')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('amr_names', ['nexus1']),
            ]
        )

        # Get the names of the AMRs from the parameter server
        self.amr_names = self.get_parameter('amr_names').value

        # Docking poses for the workstations
        w1_docking_pose = Point()
        w1_docking_pose.position.x, w1_docking_pose.position.y, w1_docking_pose.position.z = 1, 1, 0 # x, y, yaw

        w2_docking_pose = Point()
        w2_docking_pose.position.x, w2_docking_pose.position.y, w2_docking_pose.position.z = 1, 2, math.pi # x, y, yaw
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

        self.vision_locations_subscriber = self.create_subscription(
            Vision,
            'amr_vision_locations', # Consider naming to something better - maybe 'amr_vision_locations'
            self.vision_locations_handler,
            10 
        )
        
        # Attributes that get updated
        self.system_state = SystemState()
        self.schedule = Schedule()
        self.vision_locations = Vision()

    def vision_locations_handler(self, msg):
        # Update local vision locations attribute
        self.vision_locations = msg
        # Possibly just use the locations of the amrs in the system state instead of the vision locations??
        amr_locations = np.reshape(self.vision_locations.part_location,(int(len(self.vision_locations.part_location)/3),3))

        # Check if the AMRs are close enough to any of the docking poses
        for i, amr_location in enumerate(amr_locations): # For every amr location
            for j, docking_pose in enumerate(self.workstation_docking_poses): # For every possible docking pose
                distance = math.sqrt((amr_location[0] - docking_pose.position.x)**2 + (amr_location[1] - docking_pose.position.y)**2)
                # If the AMR is close enough to the docking pose
                if distance < 0.1 and amr_location[2] - docking_pose.position.z < 0.1: # There could be a better way of checking that the robot is at the docking location. 
                    # The AMR is close enough and aligned with the docking pose of the workstation
                    parts_schedule = self.schedule.parts
                    subprocesses_schedule = self.schedule.subprocesses
                    workstations_schedule = self.schedule.workstations
                    
                    # Check if the part on that robot is supposed to be processed at that workstation
                    amr_id = self.vision_locations.part_id[i] # "Part ID" here is actually the AMR ids
                    for part in parts_schedule:
                        if part.location == amr_id: # Select the part that is on the AMR
                            index = parts_schedule.index(part) # Find the index (in the schedule) of the part that is on the AMR
                            workstation = workstations_schedule[index] # Match that index with the workstation in the schedule
                            
                            # Check if the workstation is available
                            for state_workstation in self.system_state.workstations: # State workstation contains the actual state of the workstations
                                if state_workstation.id == workstation.id: # Match the actual workstation id with the workstation id in the schedule
                                    if state_workstation.status == 'FREE':
                                        # Send a command to the AMR to move to the workstation
                                        self.send_workstation_command(workstation.id, "PART_INPUT")
                                    elif state_workstation.status == 'BUSY':
                                        self.send_workstation_command(workstation.id, "PART_OUTPUT")
                                    else:
                                        # The workstation is busy or broken
                                        pass
                    # If here, there is no part on the AMR that is at a docking location.
                    pass
        # If here, there are no AMRs close enough to any of the docking poses


                            

    def send_workstation_command(self, workstation_id, command):  
        # Create a Workstation message
        workstation_command = WorkstationCommand()
        workstation_command.id = command


        

    
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

    
    