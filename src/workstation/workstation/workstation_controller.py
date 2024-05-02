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

        # Attributes that get updated
        self.system_state = SystemState()
        self.workstations_state = []
        self.schedule = Schedule()
        self.vision_locations = Vision()

        # Get the names of the AMRs from the parameter server
        self.amr_names = self.get_parameter('amr_names').value

        # Directly copied from fleet controller
        self.workstation_goal_poses = {  # CHANGE TO ACTUAL VALUES
            'workstation1': [0.8, 1.6903, np.pi-0.72],
            # 'workstation2': [1.0093, -0.6753, -0.72],
            'workstation2': [2.88, 1.75, 0.72],
            'workstation3': [1.77, 1.75, 1.57]
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
            '/system_state',
            self.system_state_handler,
            10
        )

        self.schedule_subscriber = self.create_subscription(
            Schedule,
            '/schedule',
            self.schedule_handler,
            10
        )

        self.system_state_publisher = self.create_publisher(
            SystemState,
            '/system_state',
            10
        )

        self.vision_locations_subscriber = self.create_subscription(
            Vision,
            '/amr_vision_locations', # Consider naming to something better - maybe 'amr_vision_locations'
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

        self.init_workstations()

    def init_workstations(self):
        # Create some fake workstations for now
        workstation1 = Workstation()
        workstation1.workstation_id = 1
        workstation1.state = 'FREE'
        workstation1.available_operations = ['DRILLING']
        workstation2 = Workstation()
        workstation2.workstation_id = 2
        workstation2.state = 'FREE'
        workstation2.available_operations = ['MILLING']
        workstation3 = Workstation()
        workstation3.workstation_id = 3
        workstation3.state = 'FREE'
        workstation3.available_operations = ['LOADING','UNLOADING']

        self.workstations_state = [workstation1, workstation2, workstation3]
        self.system_state.workstations = self.workstations_state
        self.system_state_publisher.publish(self.system_state)
        self.get_logger().info('Workstations have been initialized')

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
                # self.get_logger().info(f"distance: {distance}, YAW: {amr_location[2]}")
                # If the AMR is close enough to the docking pose
                if distance <= 0.09 and amr_location[2] - docking_pose.z <= 0.12: # There could be a better way of checking that the robot is at the docking location. 
                    # The AMR is close enough and aligned with the docking pose of the workstation
                    self.get_logger().info("WE ARE AT A WORKSTATION YESSSSS")
                    time.sleep(1)
                    parts_schedule = self.schedule.parts
                    subprocesses_schedule = self.schedule.subprocesses
                    workstations_schedule = self.schedule.workstations
                    # self.send_workstation_command(2, "INPUT")

                    # Two scenarios:
                    #  - The robot could be dropping off a part at the workstation. 
                    #  - The robot could be picking up a part from the workstation.
                    
                    
                    amr_id = self.vision_locations.part_id[i] # "Part ID" here is actually the AMR ids
                    for part in parts_schedule:
                        if part.location.startswith("workstation"): # If the part is at a workstation, the robot is coming to pick it up
                            index = parts_schedule.index(part)
                            state_workstation = self.workstations_state # The workstation that the part is at

                            # for state_workstation in self.system_state.workstations: # State workstation contains the actual state of the workstations
                                # if state_workstation.workstation_id == workstation.workstation_id: # Match the actual workstation id with the workstation id in the schedule
                            if state_workstation.state == 'BUSY':
                                # If the workstation is free, the robot is coming to pick up a part
                                self.send_workstation_command(state_workstation.workstation_id, "OUTPUT")
                                self.workstations_state[state_workstation.workstation_id-1].state = 'FREE'
                                break
                            # elif state_workstation.state == 'FREE':
                                
                            #     self.send_workstation_command(workstation.id, "INPUT")
                            #     break
                            else:
                                self.get_logger().info("WORKSTATION IS FREE??? OR BROKEN")
                                # The workstation is busy or broken
                                pass

                        if part.location == "amr"+str(amr_id): # If the part is on the robot at the docking location
                            index = parts_schedule.index(part)
                            workstation = workstations_schedule[index]
                            if workstation.workstation_id == j+1: # If the workstation id in the schedule matches the workstation id of the docking pose that matches (to avoid running this as soon as a part is placed)
                                for state_workstation in self.workstations_state: # State workstation contains the actual state of the workstations
                                    if state_workstation.workstation_id == workstation.workstation_id: # Match the actual workstation id with the workstation id in the schedule
                                        if state_workstation.state == 'FREE':
                                            # If the workstation is free, and the part is on the robot, the robot is coming to drop off a part
                                            self.send_workstation_command(workstation.workstation_id, "INPUT")
                                            self.workstations_state[workstation.workstation_id-1].state = 'BUSY' # Workstation is now busy, processing the part.
                                            break
                                        # elif state_workstation.state == 'BUSY':
                                        #     self.send_workstation_command(workstation.id, "INPUT")
                                        #     break
                                        else:
                                            # The workstation is busy or broken
                                            self.get_logger().info("WORKSTATION IS BUSY OR BROKEN")
                                            pass
                    # If here, there is no part on the AMR that is at a docking location.
                    pass
        self.system_state.workstations = self.workstations_state
        self.system_state_publisher.publish(self.system_state)

    def send_workstation_command(self, workstation_id, command):  
        if workstation_id == 1:
            workstation_command = WorkstationCommand()
            workstation_command.command = command
            self.workstation1_workstation_command_publisher.publish(workstation_command)
            self.get_logger().info(f"Published workstation {workstation_id} command: {command}")

        elif workstation_id == 2:
            workstation_command = WorkstationCommand()
            workstation_command.command = command
            self.workstation2_workstation_command_publisher.publish(workstation_command)
            self.get_logger().info(f"Published workstation {workstation_id} command: {command}")

        elif workstation_id == 3:
            workstation_command = WorkstationCommand()
            workstation_command.command = command
            self.workstation3_workstation_command_publisher.publish(workstation_command)
            self.get_logger().info(f"Published workstation {workstation_id} command: {command}")
        else:
            self.get_logger().info("INVALID WORKSTATION ID")
    
    def system_state_handler(self, msg):
        self.system_state = msg
        if (len(self.system_state.workstations) == 0):
            self.system_state.workstations = self.workstations_state
            # Publish the system state
            self.system_state_publisher.publish(self.system_state)

    
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

    
    