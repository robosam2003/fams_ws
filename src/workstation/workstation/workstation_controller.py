#!/usr/bin/env python3
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Schedule, Location, Vision, WorkstationCommand
from rosidl_runtime_py import *
import math
import numpy as np
import time


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

        # Directly copied from fleet controller
        self.workstation_goal_poses = {  # CHANGE TO ACTUAL VALUES
            'workstation1': [0.8, 1.6903, np.pi-0.72],
            # 'workstation2': [1.0093, -0.6753, -0.72],
            'workstation2': [2.7793, 1.6903, 0.72],
        }

        # Docking poses for the workstations
        w1_docking_pose = Point()
        w1_docking_pose.x, w1_docking_pose.y, w1_docking_pose.z = self.workstation_goal_poses['workstation1']

        w2_docking_pose = Point()
        w2_docking_pose.x, w2_docking_pose.y, w2_docking_pose.z = self.workstation_goal_poses['workstation2']
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

        self.workstation1_workstation_command_publisher = self.create_publisher(
            WorkstationCommand,
            '/workstation1/WorkstationCommand',
            10
        )

        self.workstation2_workstation_command_publisher = self.create_publisher(
            WorkstationCommand,
            '/workstation2/WorkstationCommand',
            10
        )

        self.workstation3_workstation_command_publisher = self.create_publisher(
            WorkstationCommand,
            '/workstation3/WorkstationCommand',
            10
        )
        
        # Attributes that get updated
        self.system_state = SystemState()
        self.schedule = Schedule()
        self.vision_locations = Vision()

    def vision_locations_handler(self, msg):
        # self.get_logger().info("VISION LOCATIONS HANDLER")
        # Update local vision locations attribute
        self.vision_locations = msg
        # Possibly just use the locations of the amrs in the system state instead of the vision locations??
        amr_locations = np.reshape(self.vision_locations.part_location,(int(len(self.vision_locations.part_location)/3),3))
        for i in range(len(amr_locations)):
            map_odom_tf = [1.77, 1.015, 0]
            amr_locations[i][0] = amr_locations[i][0] + map_odom_tf[0]
            amr_locations[i][1] = -amr_locations[i][1] + map_odom_tf[1]
            amr_locations[i][2] = -amr_locations[i][2] + map_odom_tf[2]


        # Check if the AMRs are close enough to any of the docking poses
        for i, amr_location in enumerate(amr_locations): # For every amr location
            for j, docking_pose in enumerate(self.workstation_docking_poses): # For every possible docking pose
                distance = math.sqrt((amr_location[0] - docking_pose.x)**2 + (amr_location[1] - docking_pose.y)**2)
                self.get_logger().info(f"distance: {distance}, YAW: {amr_location[2]}")
                # If the AMR is close enough to the docking pose
                if distance <= 0.15 and amr_location[2] - docking_pose.z <= 0.18: # There could be a better way of checking that the robot is at the docking location. 
                    # The AMR is close enough and aligned with the docking pose of the workstation
                    self.get_logger().info("WE ARE AT A WORKSTATION YESSSSS")
                    time.sleep(1)
                    parts_schedule = self.schedule.parts
                    subprocesses_schedule = self.schedule.subprocesses
                    workstations_schedule = self.schedule.workstations
                    self.send_workstation_command(2, "INPUT")
                    
                    # Check if the part on that robot is supposed to be processed at that workstation
                    amr_id = self.vision_locations.part_id[i] # "Part ID" here is actually the AMR ids
                    for part in parts_schedule:
                        if part.location == "amr"+amr_id: # Select the part that is on the AMR
                            index = parts_schedule.index(part) # Find the index (in the schedule) of the part that is on the AMR
                            workstation = workstations_schedule[index] # Match that index with the workstation in the schedule
                            
                            # Check if the workstation is available
                            for state_workstation in self.system_state.workstations: # State workstation contains the actual state of the workstations
                                if state_workstation.id == workstation.id: # Match the actual workstation id with the workstation id in the schedule
                                    if state_workstation.status == 'FREE':
                                        # Send a command to the AMR to move to the workstation
                                        self.send_workstation_command(workstation.id, "INPUT")
                                    elif state_workstation.status == 'BUSY':
                                        self.send_workstation_command(workstation.id, "OUTPUT")
                                    else:
                                        # The workstation is busy or broken
                                        pass
                        # elif part.location == "workstation"+
                    # If here, there is no part on the AMR that is at a docking location.
                    pass
        # If here, there are no AMRs close enough to any of the docking poses

    def send_workstation_command(self, workstation_id, command):  
        if workstation_id == 1:
            workstation_command = WorkstationCommand()
            workstation_command.id = command
            self.workstation1_workstation_command_publisher.publish(workstation_command)
            self.get_logger().info(f"Published workstation {workstation_id} command: {command}")

        elif workstation_id == 2:
            workstation_command = WorkstationCommand()
            workstation_command.id = command
            self.workstation2_workstation_command_publisher.publish(workstation_command)
            self.get_logger().info(f"Published workstation {workstation_id} command: {command}")

        elif workstation_id == 3:
            workstation_command = WorkstationCommand()
            workstation_command.id = command
            self.workstation3_workstation_command_publisher.publish(workstation_command)
            self.get_logger().info(f"Published workstation {workstation_id} command: {command}")
        else:
            self.get_logger().info("INVALID WORKSTATION ID")


        

    
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

    
    