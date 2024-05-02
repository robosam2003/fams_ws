#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

from fams_interfaces.msg import Job, SubProcess, Part, SystemState, Workstation, Location, WorkstationCommand, Vision, RFID, Mover6Control
#from workstation.workstation import Location
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np
import math
import time
import serial
import ikpy.chain

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

"""
To launch a workstation, with a mover6, use:
Workstation 2:
    ros2 launch mover6_description mover6_launch.py robot_name:=workstation2 initial_base_link_pos:="3.31 2.05 -1.57079"
"""


class Workstation(Node):
    def __init__(self):
        super().__init__('workstation')
        self.get_logger().info('Workstation node has been started')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('workstation_name', 'workstationX'),
            ]
        )

        self.workstationName = self.get_parameter('workstation_name').value
        self.workstation_id = int(self.workstationName[-1])

        # Subscriptions
        self.SystemStateSubscription=self.create_subscription(SystemState,
                                                                '/system_state', # absolute path to the topic
                                                                self.system_state_callback,
                                                                10 )
        self.vision_locations_subscription = self.create_subscription(Vision,
                                                                'VisionLocations', # Namespaced
                                                                    self.vision_callback,
                                                                    10)
        
        self.workstationCommandSubscription=self.create_subscription(WorkstationCommand,
                                                                'WorkstationCommand', # Namespaced
                                                                self.workstation_cmd_callback,
                                                                10 )        
        
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states', # Namespaced
            self.joint_state_callback, 
            10
        )

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
        self.joint_states = JointState()
        self.workstation_id = 0
        self.available_operations = []
        self.status = 'FREE'  # Add a 'status' attribute with default value 'FREE'


        self.my_chain = ikpy.chain.Chain.from_urdf_file("src/mover6_description/src/description/CPRMover6WithGripperIKModel.urdf.xacro")

        
        self.place_location = [0.36, -0.05, 0.15]
        self.rfid_scan_location = [-0.05, 0.41, 0.15]
        self.idle_position = [0.30, -0.30, 0.30]
        self.previous_pickup_location = [0.3, 0.3, 0.3]
        self.zero_point = [0.34594893, 0.01, 0.44677551]
        self.current_point = None

        if self.workstation_id == 1:
            self.rfid_scan_location = [-0.05, -0.41, 0.15] # Scan the other way please


        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
        self.rfid_tag=0

        self.rfid_publisher = self.create_publisher(
            RFID,
            'rfid_tag',
            10
        )
        
    
        self.rfid_timer=self.create_timer(0.5,
                                          self.read_rfid)

    def system_state_callback(self,msg):
        self.system_state=msg

    def joint_state_callback(self, msg):
        # self.get_logger().info('Received Joint States')
        self.joint_states = msg
    
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
        part_location[0] = part_location[0] - 0.07
        # part_location = [0.3, 0.3, 1.57] # FAKE LOCATION
        self.get_logger().info(f'Part Location: {part_location}')
        self.previous_pickup_location = part_location
        
        delay_between_operations = 1
        self.close_gripper()
        time.sleep(delay_between_operations)
        self.open_gripper()
        time.sleep(delay_between_operations)
        self.move_robot(part_location[0], part_location[1], 0.3)
        time.sleep(delay_between_operations)
        self.move_robot(part_location[0], part_location[1], 0.15) # Move down to over part
        time.sleep(delay_between_operations)
        self.close_gripper()
        time.sleep(delay_between_operations)
        self.move_robot(part_location[0], part_location[1], 0.3)
        time.sleep(delay_between_operations)
        self.move_robot(self.rfid_scan_location[0], self.rfid_scan_location[1], self.rfid_scan_location[2])


    def handle_part_output(self,msg):
        # self.read_rfid()
        #update x y attributes when receive a visionLocations msg

        # locations = self.vision_locations.part_location
        # location_array_2D=np.reshape(locations,(int(len(locations)/3),3))
        # locations = self.vision_locations.part_location
        # print(locations)
        # location_array_2D=np.reshape(locations,(int(len(locations)/3),3))
        
        delay_between_operations = 1
        self.move_robot(self.rfid_scan_location[0], self.rfid_scan_location[1], self.rfid_scan_location[2]+0.2)
        time.sleep(delay_between_operations)
        self.move_robot(self.previous_pickup_location[0], self.previous_pickup_location[1], 0.3)
        time.sleep(delay_between_operations)
        self.move_robot(self.previous_pickup_location[0], self.previous_pickup_location[1], 0.15)
        time.sleep(delay_between_operations)
        self.open_gripper()
        time.sleep(delay_between_operations)
        self.move_robot(self.previous_pickup_location[0], self.previous_pickup_location[1], 0.3)
        time.sleep(delay_between_operations)
        self.move_robot(self.zero_point[0], self.zero_point[1], self.zero_point[2])

        # Update the part location to amr0
        msg = RFID()
        msg.workstation_id = 0 # 0 indicates that this is a loading onto amr (output) event
        msg.rfid_code = self.rfid_tag
        self.rfid_publisher.publish(msg)
    
    def calculate_unit_vector(self, current_point, target_point):
        vector = np.array(target_point) - np.array(current_point)
        magnitude = np.linalg.norm(vector)
        unit_vector = vector / magnitude
        return unit_vector, magnitude

    def move_robot(self,x,y,z):
        overall_target_point = [x, y, z]

        link_angles = [0,
                   self.joint_states.position[0],
                   self.joint_states.position[1],
                   self.joint_states.position[2],
                   self.joint_states.position[3],
                   self.joint_states.position[4],
                   self.joint_states.position[5],
                   0, 0]
        
        self.get_logger().info(f"Link angles: {link_angles}")
        if self.current_point is None:
            self.current_point = self.my_chain.forward_kinematics(link_angles)
            self.current_point = self.current_point[:3, 3]
            self.get_logger().info(f"Current point: {self.current_point}")
        
        
        num_points = 30
        for i in range(0, num_points):
            # Calculate unit vector between the current and target points
            unit_vector, magnitude = self.calculate_unit_vector(self.current_point, overall_target_point)
            target_point = self.current_point + unit_vector * (magnitude/num_points) * (i+1)
            self.get_logger().info("Target point: " + str(target_point))
            mover6_pose_msg = Pose()
            mover6_pose_msg.position.x = target_point[0]
            mover6_pose_msg.position.y = target_point[1]
            mover6_pose_msg.position.z = target_point[2]

            self.mover6Publisher.publish(mover6_pose_msg)
            self.get_logger().info("Published pose to mover6: " + str(mover6_pose_msg.position.x) + ", " + str(mover6_pose_msg.position.y) + ", " + str(mover6_pose_msg.position.z))
            time.sleep(0.06)
            self.current_point = target_point # Update the current point to the target point (it's moved)
        
    def vision_callback(self, msg):
        # self.get_logger().info('Received Vision Locations')
        if len(msg.part_location) == 0:
            self.get_logger().warn('No parts detected')
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
        # msg = RFID()
        a = self.ser.readline().strip()  # Remove leading/trailing whitespace
        rfid_code = a.decode('utf-8')  # bytes to string
        if len(a) > 0:
            self.get_logger().info(rfid_code)
            #var = var[len("Tag ID:"):].strip()
            if rfid_code != None:
                self.rfid_tag=rfid_code
                print(self.rfid_tag)
                print("update current rfid tag")
                msg = RFID()
                msg.workstation_id = self.workstation_id
                msg.rfid_code = self.rfid_tag
                self.rfid_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    workstation = Workstation()
    rclpy.spin(workstation)
    workstation.destroy_node()
    rclpy.shutdown()

    