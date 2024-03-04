import MoverJoint
import PCanBus
import time
import rclpy
from rclpy.node import Node
import subprocess
from sensor_msgs.msg import JointState

"""
Initialisation Commands:
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
    tics_per_degree = [-65.87,
                        -65.87,
                        65.87,
                        -69.71,
                        3.2,
                        3.2]   
    
    MIN_MAX_POS_DEG = [(-150, 150),
                        (-30, 60),
                        (-110, 60),
                        (-130, 150),
                        (-90, 90),
                        (-130, 130)]

    def __init__(self):
        # Need to run the CAN interface before we do anything
        subprocess.run(["sudo", "modprobe", "peak_usb"])    
        subprocess.run(["sudo", "modprobe", "peak_pci"])
        subprocess.run(["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "500000"])
        time.sleep(1)

        # Node init
        super().__init__('mover6')

        # Setup robot joint state publisher subscription
        self.joint_positions_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
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

        self.main_loop()


    def main_loop(self):  # The main loop maintains a 20Hz control cycle, necessary for the motor control boards (they will give a CAN error if not)
        # =======================================
        # ================ Setup ================
        # =======================================
        max_joint = 1 # Just for testing, don't want to move all the joints at once while developing. 
        # 2. Start the main loop and send position command cyclically 
        for joint in self.joints[0:max_joint]:
            # Single pulse to mark position
            joint.set_position(joint, joint.position_deg)
            rec_msg = self.can_bus.recv()

            # Parse the message and update the joint position
            joint.update_position(rec_msg)
            time.sleep(2/1000) # 2ms wait between each joint

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
            # self.set_zero_position(joint)
            time.sleep(2/1000)
        time.sleep(0.5)
            
        # 5. Enable all joints
        for joint in self.joints[0:max_joint]:
            self.enable()
            time.sleep(2/1000)    

        i = 0
        reference = -45 # This is in degrees
        tick_tock = -1
        # reference = int(reference / (122146/-20000))
        position_command = reference
        # ===========================================
        # ================ Main Loop ================
        # ===========================================
        while True:                      
            for joint in range(1, max_joint+1):
                self.set_position_32bit(joint, reference)
                rec_msg = self.recv()
                # # Parse the message and update the joint position
                joint_pos = rec_msg.position
                self.joint_positions[joint-1] = joint_pos
                print("Joint", joint, "is at position: ", joint_pos)

                time.sleep(2/1000) 
            i += 1
            time.sleep(1/20) # 20Hz

    def shutdown(self):
        self.can_bus.shutdown()

    def __del__(self):
        self.can_bus.shutdown()


def main(args=None):
    rclpy.init(args=args)
    mover6_node = mover6()
    rclpy.spin(mover6_node)
    mover6_node.shutdown()
    rclpy.shutdown()
