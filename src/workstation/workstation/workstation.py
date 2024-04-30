#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Location, WorkstationCommand, Vision, RFID, Mover6Control
#from workstation.workstation import Location
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import numpy as np
import math
import time
import serial

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
        self.workstationName='Workstation1'
        self.get_logger().info('Workstation node has been started')

        # Subscriptions
        self.SystemStateSubscription=self.create_subscription(SystemState,
                                                                '/system_state', # absolute path to the topic
                                                                self.system_state_callback,
                                                                10 )
        self.vision_locations_subscription = self.create_subscription(Vision,
                                                                '/workstation2/VisionLocations', # Absolute path to the topic - we are in a namespace! 
                                                                    self.vision_callback,
                                                                    10)
        
        self.workstationCommandSubscription=self.create_subscription(WorkstationCommand,
                                                                'WorkstationCommand', # Namespaced
                                                                self.workstation_cmd_callback,
                                                                10 )        

        # Publishers
        self.mover6Publisher=self.create_publisher(Pose,
                                                   'mover6_goal_pose',
                                                   10)

        self.mover6_gripper_publisher = self.create_publisher (
            Mover6Control,
            'mover6_control',
            10
        )
        # self.RFID_publisher = self.create_publisher(RFID, "RFID_Topic", 10)
        # know how to determine if state= free/busy
        self.workstation_publisher = self.create_publisher(
            SystemState,
            'system_state',
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.vision_locations = Vision()
        self.system_state = SystemState()
        self.workstation_id = 0
        self.available_operations = []
        self.status = 'FREE'  # Add a 'status' attribute with default value 'FREE'

        self.place_location = [0.36, -0.05, 0.15]
        # self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
        # self.read_raspberry_pi_()
        self.rfid_tag=0
        
    
        # self.rfid_timer=self.create_timer(1,
        #                                   self.read_rfid)

    def system_state_callback(self,msg):
        self.system_state=msg
    
    def open_gripper(self):
        # Open the gripper
        msg = Mover6Control()
        msg.command = "GRIPPER OPEN"
        self.mover6_gripper_publisher.publish(msg)
        self.get_logger().info("Gripper opened")

    def close_gripper(self):
        # Close the gripper
        msg = Mover6Control()
        msg.command = "GRIPPER CLOSED"
        self.mover6_gripper_publisher.publish(msg)
        self.get_logger().info("Gripper closed")


    def handle_part_input(self):    
        self.get_logger().info("Handling Part Input")
        locations = self.vision_locations.part_location
        location_array_2D=np.reshape(locations,(int(len(locations)/3),3))
        
        self.get_logger().info('Excecuting Part Input')
        part_location = location_array_2D[0]
        part_location = [float(i) for i in part_location]
        
        delay_between_operations = 5
        # Above part
        # self.move_robot(part_location[0],part_location[1],0.4) # Above part on AMR
        # time.sleep(delay_between_operations)
        # self.move_robot(part_location[0],part_location[1],0.2) # 
        # time.sleep(delay_between_operations)
        self.close_gripper()
        time.sleep(delay_between_operations)
        self.move_robot(self.place_location[0], self.place_location[1], self.place_location[2]+0.2)
        time.sleep(delay_between_operations)
        self.move_robot(self.place_location[0], self.place_location[1], self.place_location[2]) 
        time.sleep(delay_between_operations)
        self.open_gripper()
        time.sleep(delay_between_operations)
        
        # parts=msg.parts
        # for i in parts:
        #     if parts[i].part_id==self.msg.rfid_code:
        #         #which part’s loc to assign workstation to
        #         parts[i].location=self.workstationName
        #         print("part location changed to: ", parts[i].location)

        #update part's current subprocess

        # workstation1=Workstation
        # workstation1.state='busy'


    def handle_part_output(self,msg):
        self.read_rfid()
        #update x y attributes when receive a visionLocations msg

        locations = self.vision_locations.part_location
        location_array_2D=np.reshape(locations,(int(len(locations)/3),3))
        locations = self.vision_locations.part_location
        print(locations)
        location_array_2D=np.reshape(locations,(int(len(locations)/3),3))
        
        # from bot to floor
        self.move_robot(self,location_array_2D[0],location_array_2D[1],0.2)
        time.sleep(1)
        self.move_robot(self,location_array_2D[0],location_array_2D[1],0.3)
        time.sleep(1)
        self.move_robot(self,0.25,0.25,0.3)
        time.sleep(1)
        self.move_robot(self,0.25,0.25,0.1)

        
        parts=msg.parts
        for i in parts:
            if parts[i].part_id==self.msg.rfid_code:
                #which part’s loc to assign workstation to
                parts[i].location=self.workstationName
                print("part location changed to: ", parts[i].location)
        
        
#function move to box 
#find out which id is where
        part_id=self.vision_locations.part_id
        print(
        part_id,
        location_array_2D[0])
        self.get_logger().info(part_id,
        location_array_2D[0])
        

        #SAM: update part's current subprocess


        workstation1=Workstation
        workstation1.state='free'

        
    def move_robot(self,x,y,z):
        # generic fx 3d coord ip, move robot there
        mover6_pose_msg = Pose()
        mover6_pose_msg.position.x = x
        mover6_pose_msg.position.y = y
        mover6_pose_msg.position.z = z
        # q = quaternion_from_euler(theta,phi,psi)
        # mover6_pose_msg.orientation.x = q[0]
        # mover6_pose_msg.orientation.y = q[1]
        # mover6_pose_msg.orientation.z = q[2]
        # mover6_pose_msg.orientation.w = q[3]

        self.mover6Publisher.publish(mover6_pose_msg)
        self.get_logger().info("Published pose to mover6")
        
        
    def vision_callback(self, msg):
        self.get_logger().info('Received Vision Locations')
        if len(msg.part_location) == 0:
            self.get_logger().info('No parts detected')
            return
        # else
        self.vision_locations = msg
        # self.handle_part_output(msg)

    def workstation_cmd_callback(self, msg):
        command = msg.command
        self.get_logger().info(f'Received Command: {command}' )
        if command == "INPUT":
            self.handle_part_input()
        elif command == "OUTPUT":
            self.handle_part_output(msg)
    
    def read_rfid(self):
        msg = RFID()
        a = self.ser.readline().strip()  # Remove leading/trailing whitespace
        msg.rfid_code = a.decode('utf-8')  # bytes to string
        if len(a) > 0:
            self.get_logger().info(msg.rfid_code)
            #var = var[len("Tag ID:"):].strip()
            if msg.rfid_code != None:
                self.rfid_tag=msg.rfid_code
                print(self.rfid_tag)
                print("update current rfid tag")

def main(args=None):
    rclpy.init(args=args)
    workstation = Workstation()
    rclpy.spin(workstation)
    workstation.destroy_node()
    rclpy.shutdown()

    