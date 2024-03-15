import MoverJoint
import PCanBus
import time
import rclpy
from rclpy.node import Node
import subprocess
from fams_interfaces.msg import Mover6Control

"""
Before running this program, run: 
    sudo modprobe peak_usb
    sudo modprobe peak_pci
    sudo ip link set can0 up type can bitrate 500000

"""

def _get_message(msg):
    print("Received: ", msg)
    return msg
 
    

class mover6(Node):
    # CONSTANTS
    MAX_LAG = 0  # Disabled
    # tics_per_degree = [-65.87,
    #                     -65.87,
    #                     65.87,
    #                     -69.71,
    #                     3.2,
    #                     3.2]
    # tics_per_degree = [-256,
    #                     -256,
    #                     256,
    #                     -256,
    #                     15,
    #                     15]
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

        # Node init
        super().__init__('mover6')

        # Setup robot joint state publisher subscription
        self.joint_positions_subscription = self.create_subscription(
            Mover6Control,
            'mover6_control',
            self.mover6_control_callback,
            10
        )

        # Setup robot joint state publisher
        self.joint_positions_publisher = self.create_publisher(
            Mover6Control,
            'mover6_state',
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
        
        self.desired_joint_angles = [0, 0, 30, 90, 0, 0] # This is the desired joint angles - initially zero, these will be set with the subscriber callback
        # Setup a timer to run the main loop
        self.max_joint = 6
        self.setup()
        timer = self.create_timer(0.05, self.main_loop)  # 20Hz

        # self.main_loop()

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
            joint.set_pos_pid(0.1, 0.1, 0)
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
            j.set_position(reference)
            rec_msg = self.can_bus.recv()
            # # Parse the message and update the joint position
            joint_pos = rec_msg.position
            self.joints[joint-1].update_position(rec_msg)
            
            print("Joint", joint, "is at position: ", j.current_position_deg, " degrees")
            time.sleep(2/1000) 
        # Publish the joint positions
        msg = Mover6Control()   
        msg.joint_angles = [float(j.current_position_deg) for j in self.joints]
        self.joint_positions_publisher.publish(msg)


    def mover6_control_callback(self, msg):
        if msg.command == "ZERO":
            joint_no = msg.joint_no
            self.joints[joint_no-1].set_zero_position()
            time.sleep(2/1000)
            # Reenable the joint
            self.joints[joint_no-1].enable()
            time.sleep(2/1000)            
        # Set the desired joint angles to the received joint angles
        self.desired_joint_angles = msg.joint_angles
        # Saturate the desired joint angles to the joint limits
        for i in range(0, self.num_joints):
            if self.desired_joint_angles[i] > self.MIN_MAX_POS_DEG[i][1]:
                self.desired_joint_angles[i] = self.MIN_MAX_POS_DEG[i][1]
            elif self.desired_joint_angles[i] < self.MIN_MAX_POS_DEG[i][0]:
                self.desired_joint_angles[i] = self.MIN_MAX_POS_DEG[i][0]
        print("Received: ", self.desired_joint_angles)
        # print(("RECIEVED \n")*100)
        


def main(args=None):
    rclpy.init(args=args)
    mover6_node = mover6()
    rclpy.spin(mover6_node)
    mover6_node.shutdown()
    rclpy.shutdown()
