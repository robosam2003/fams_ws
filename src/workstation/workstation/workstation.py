#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Location, WorkstationCommand, Vision
#from workstation.workstation import Location
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import numpy as np
import math
import time
#from machine_vision import eStop
#from mover6 import mover6


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class Workstation(Node):
    def __init__(self):
        super().__init__('workstation')
        self.get_logger().info('Workstation node has been started')

        self.workstation_id = 0
        self.available_operations = []
        self.status = 'FREE'  # Add a 'status' attribute with default value 'FREE'
        
        self.subscription = self.create_subscription(Vision,
                                                      'VisionLocations',
                                                        self.vision_callback,
                                                          10)
        self.x = 0.0
        self.y = 0.0
        self.vision_locations = Vision()
        self.workstationCommandSubscription=self.create_subscription(String,
                                                                     'WorkstationCommand',
                                                                     self.workstation_cmd_callback,
                                                                     10 )
    
        
        #know how to determine if state= free/busy
        # self.workstation_publisher = self.create_publisher(
        #     SystemState,
        #     'system_state',
        #     10
        # )
        self.mover6Publisher=self.create_publisher(Pose,
                                                   'mover6_goal_pose',
                                                   10)

        # SystemState1=SystemState
        # # Workstations State
        

        # # Part State
        #     workstation1.parts

        # # Mobile Fleet State
        #     workstation1.mobile_robots
    def handle_part_input(self):
        #publish Workstation msg
        pass


    def handle_part_output(self):
        #update x y attributes when receive a visionLocations msg
        locations = self.vision_locations.part_location
        location_array_2D=np.reshape(locations,(int(len(locations)/3),3))

        #from bot to floor
        self.move_robot(self,location_array_2D[0],location_array_2D[1],0.2,1.57,0,0)
        self.move_robot(self,location_array_2D[0],location_array_2D[1],0.3,1.57,0,0)
        self.move_robot(self,0.25,0.25,0.3,1.57,0,0)
        self.move_robot(self,0.25,0.25,0.1,1.57,0,0)


        
#function move to box 
#find out which id is where
        part_id=self.vision_locations.part_id
        print(
        part_id,
        location_array_2D[0])
        self.get_logger().info(part_id,
        location_array_2D[0])
        
        workstation1=Workstation
        workstation1.state='free'


    
    def move_robot(self,x,y,z, theta,phi,psi):
        #generic fx 3d coord ip, move robot there
      
        
        
        mover6_pose_msg = Pose()
        mover6_pose_msg.position.x = x
        mover6_pose_msg.position.y = y
        mover6_pose_msg.position.z = z
        q = quaternion_from_euler(theta,phi,psi)
        mover6_pose_msg.orientation.x = q[0]
        mover6_pose_msg.orientation.y = q[1]
        mover6_pose_msg.orientation.z = q[2]
        mover6_pose_msg.orientation.w = q[3]

        self.mover6Publisher.publish(mover6_pose_msg)
        self.get_logger().info("Published pose to mover6")
        time.sleep(2)
        
        
    def vision_callback(self, msg):
        self.vision_locations = msg
        print("tftgbhjn")
        self.handle_part_output()


    def workstation_cmd_callback(self, command:WorkstationCommand,msg):
        self.get_logger().info('Received Command: ' )
        if command == "PART INPUT":
            self.handle_part_input()
        elif command == "PART OUTPUT":
            self.handle_part_output()
    
    
    

def main(args=None):
    rclpy.init(args=args)
    workstation_controller = Workstation()
    rclpy.spin(workstation_controller)
    workstation_controller.destroy_node()
    rclpy.shutdown()

    