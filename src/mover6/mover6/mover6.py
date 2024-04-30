import MoverJoint
import PCanBus
import time
import rclpy
from rclpy.node import Node
import subprocess
from fams_interfaces.msg import Mover6Control
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PoseStamped, Pose
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import math


"""
Before running this program, run: 
    sudo modprobe peak_usb &&
    sudo modprobe peak_pci &&
    sudo ip link set can0 up type can bitrate 500000

"""


class mover6(Node):
    # CONSTANTS
    MAX_LAG = 0  # Disabled
    # tics_per_degree = [-65.87,
    #                     -65.87,
    #                     65.87,
    #                     -69.71,
    #                     3.2,
    #                     3.2]

    tics_per_degree = [2111,
                       2111,
                       2111,
                       2111,
                       3.55,
                       3.55]
    
    
    MIN_MAX_POS_DEG = [(-150, 150),
                        (-30, 60),
                        (-110, 60),
                        (-130, 150),
                        (-90, 90),
                        (-130, 130)]

    def __init__(self):
        # Need to run the CAN interface before we do anything
        # subprocess.run(["sudo", "modprobe", "peak_usb"])    
        # subprocess.run(["sudo", "modprobe", "peak_pci"])
        # subprocess.run(["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "500000"])
        # time.sleep(1)
        #         sudo modprobe peak_usb && sudo modprobe peak_pci && sudo ip link set can0 up type can bitrate 500000


        # Node init
        super().__init__('mover6')

        self.my_chain = ikpy.chain.Chain.from_urdf_file("src/mover6_description/src/description/CPRMover6WithGripperIKModel.urdf.xacro")

        # Setup robot joint state publisher subscription
        self.joint_control_subscription = self.create_subscription(
            Mover6Control,
            'mover6_control',
            self.mover6_joints_control_callback,
            10
        )

        self.pose_control_subscription = self.create_subscription(
            Pose,
            'mover6_goal_pose',
            self.mover6_pose_control_callback,
            10
        )


        # Setup robot joint state publisher
        self.joint_states_publisher = self.create_publisher(
            JointState,
            'mover6_joint_states',
            10
        )


        # 1. Initialise the CAN bus and the joints
        self.can_bus = PCanBus.PCanBus()
        self.num_joints = 6
        self.joints = [MoverJoint.MoverJoint(i+1, # Joint 1 is at index 0, etc
                                             self.can_bus,
                                             self.tics_per_degree[i],
                                             self.MIN_MAX_POS_DEG[i][0],  # Min position
                                             self.MIN_MAX_POS_DEG[i][1])  # Max position
                                             for i in range(0, self.num_joints)]  # List conprehension to create the joints
        
        self.desired_joint_angles = [0, 0, 0, 0, 0, 0] # This is the desired joint angles - initially zero, these will be set with the subscriber callback
        self.gripper_state = False
        # Setup a timer to run the main loop
        self.max_joint = 6
        self.setup()
        self.timer = self.create_timer(0.05, self.main_loop)  # 20Hz

        # self.main_loop()
        # Initialise the joint states
        self.joint_states = JointState()
        self.joint_states.name = ['Joint0', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5']
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



        

    def setup(self):   
        # =======================================
        # ================ Setup ================
        # =======================================
        
        max_joint = self.max_joint # Just for testing, don't want to move all the joints at once while developing. 
        
        # 2. Start the main loop and send position command cyclically 
        for joint in self.joints[0:max_joint]:
            # Single pulse to mark position - Send to "current" position
            joint.set_position(joint.current_position_deg)
            rec_msg = self.can_bus.recv()

            # Parse the message and update the joint position
            joint.update_position(rec_msg)
            time.sleep(2/1000) # 2ms wait between each joint command

        # 3. Reset all joints: 
        for joint in self.joints[0:max_joint]:
            joint.reset()
            time.sleep(2/1000) # 2ms wait between each joint

        # 4. Set the parameters
        for joint in self.joints[0:max_joint]:
            joint.set_max_lag(self.MAX_LAG)
            time.sleep(2/1000)
            joint.set_pos_pid(0.2, 0.1, 0)
            time.sleep(2/1000)
            joint.set_vel_pid(0.5, 0, 0)
            time.sleep(2/1000)
            joint.set_tic_scale(1)
            time.sleep(2/1000)
            joint.set_max_current(0)
            # if joint.joint_id == 5 or joint.joint_id == 6:
            #     joint.set_zero_position()
            time.sleep(2/1000)
        time.sleep(0.5)
            
        # 5. Enable all joints
        for joint in self.joints[0:max_joint]:
            joint.enable()
            time.sleep(2/1000)    
        
        self.get_logger().info("======== Mover6 setup complete ========== Entering Main loop ===========")

    def main_loop(self):  # The main loop maintains a 20Hz control cycle, necessary for the motor control boards (they will give a CAN error if not)
        # ===========================================
        # ================ Main Loop ================
        # ===========================================
        max_joint = self.max_joint # Just for testing, don't want to move all the joints at once while developing.
        for joint in range(1, max_joint+1):
            j = self.joints[joint-1]
            reference = self.desired_joint_angles[joint-1]
            # print("Desired joint angle: ", reference, " degrees")
            # reference = 120000
            j.set_position(reference, self.gripper_state)
            rec_msg = self.can_bus.recv(vervose=False)
            # # Parse the message and update the joint position
            joint_pos = rec_msg.position
            self.joints[joint-1].update_position(rec_msg)
            
            # print("Joint", joint, "is at position: ", j.current_position_deg, " degrees")
            time.sleep(2/1000) 
        # Publish the joint positions
        DEG_TO_RAD = np.pi/180
        self.joint_states.position[0:6] = [float(j.current_position_deg*DEG_TO_RAD) for j in self.joints] # Local joint states are in degrees, message is in radians for rviz
        negate_list = [-1, -1, 1, -1, 1, 1]
        self.joint_states.position[0:6] = [negate_list[i]*self.joint_states.position[i] for i in range(0, self.num_joints)]
        self.joint_states_publisher.publish(self.joint_states)

    def quaternion_to_euler(self, quat):
        # Convert the orientation to euler angles
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = np.arctan2(t3, t4)
        return X, Y, Z


    def mover6_pose_control_callback(self, msg):
        target_point = [msg.position.x, msg.position.y, msg.position.z]
        orientation_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # Convert the orientation to euler angles
        orientation_euler = [0, 0, -1] #self.quaternion_to_euler(orientation_quat)
        orientation_axis = "Z"

        # Do the inverse kinematics
        ik = self.my_chain.inverse_kinematics(target_position=target_point)
                                            #   target_orientation=orientation_euler,
                                            #   orientation_mode=orientation_axis) # , orientation_euler)  # Uncommment to include orientation
        angles = [ik[i+1] for i in range(6)] # Skip the first angle, it's for the joint from the base_link to the first joint
        angles[0] = angles[0]
        angles[1] = -angles[1]
        angles[2] = angles[2]
        angles[3] = angles[3]
        angles[4] = angles[4]
        angles[5] = angles[5]
        
    
    def mover6_pose_control_callback(self, msg):
        target_point = [msg.position.x, msg.position.y, msg.position.z]
        orientation_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # Convert the orientation to euler angles
        orientation_euler = [0, 0, -1] #self.quaternion_to_euler(orientation_quat)
        orientation_axis = "Z"

        # Do the inverse kinematics
        ik = self.my_chain.inverse_kinematics(target_position=target_point)
                                            #   target_orientation=orientation_euler,
                                            #   orientation_mode=orientation_axis) # , orientation_euler)  # Uncommment to include orientation
        angles = [ik[i+1] for i in range(6)] # Skip the first angle, it's for the joint from the base_link to the first joint
        angles[0] = angles[0]
        angles[1] = -angles[1]
        angles[2] = angles[2]
        angles[3] = angles[3]
        angles[4] = angles[4]
        angles[5] = angles[5]
        

        angles = [np.rad2deg(a) for a in angles]
        print("IK angles: ", angles)
        # Set the desired joint angles to calculated joint angles from the IK
        self.desired_joint_angles = angles        

    def mover6_joints_control_callback(self, msg):
        # print("Received: ", msg)
        if msg.command == "GRIPPER OPEN":
            self.gripper_state = False
            print("Gripper state: ", self.gripper_state)

        elif msg.command == "GRIPPER CLOSED":
            self.gripper_state = True
            print("Gripper state: ", self.gripper_state)
        if msg.command == "ZERO":
            joint_no = msg.joint_no
            self.joints[joint_no-1].set_zero_position()
            time.sleep(2/1000)
            # Reenable the joint
            self.joints[joint_no-1].enable()
            time.sleep(2/1000)            
        # Set the desired joint angles to the received joint angles
        if len(msg.joint_angles) == 6:
            self.desired_joint_angles = msg.joint_angles
        # Saturate the desired joint angles to the joint limits
            for i in range(0, self.num_joints):
                if self.desired_joint_angles[i] > self.MIN_MAX_POS_DEG[i][1]:
                    self.desired_joint_angles[i] = self.MIN_MAX_POS_DEG[i][1]
                elif self.desired_joint_angles[i] < self.MIN_MAX_POS_DEG[i][0]:
                    self.desired_joint_angles[i] = self.MIN_MAX_POS_DEG[i][0]
            # print("Received: ", self.desired_joint_angles)
        # print(("RECIEVED \n")*100)
        


def main(args=None):
    rclpy.init(args=args)
    mover6_node = mover6()
    rclpy.spin(mover6_node)
    mover6_node.shutdown()
    rclpy.shutdown()
