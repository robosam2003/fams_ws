# An example interface node for the gui

import rclpy
from rclpy.node import Node
import Controller_ui as Controller_ui
import sys
from cv_bridge import CvBridge
from threading import Thread 
import cv2 as cv

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QMainWindow, QApplication
from rclpy.executors import MultiThreadedExecutor

from fams_interfaces.msg import Mover6Control
from sensor_msgs.msg import JointState
import numpy as np

# Interface class for the GUI
class Interface(QMainWindow, Controller_ui.Ui_MainWindow):
    def __init__(self, parent=None):
        super(Interface, self).__init__(parent)
        self.setupUi(self)
        self.rosnode = None

        # set up timers:
        self.timer1 = QTimer()
        self.timer1.timeout.connect(lambda: self.rosnode.axisButtonHandler(1, True))
        self.timer2 = QTimer()
        self.timer2.timeout.connect(lambda: self.rosnode.axisButtonHandler(1, 0))
        self.timer3 = QTimer()
        self.timer3.timeout.connect(lambda: self.rosnode.axisButtonHandler(2, True))
        self.timer4 = QTimer()
        self.timer4.timeout.connect(lambda: self.rosnode.axisButtonHandler(2, 0))
        self.timer5 = QTimer()
        self.timer5.timeout.connect(lambda: self.rosnode.axisButtonHandler(3, True))
        self.timer6 = QTimer()
        self.timer6.timeout.connect(lambda: self.rosnode.axisButtonHandler(3, 0))
        self.timer7 = QTimer()
        self.timer7.timeout.connect(lambda: self.rosnode.axisButtonHandler(4, True))
        self.timer8 = QTimer()
        self.timer8.timeout.connect(lambda: self.rosnode.axisButtonHandler(4, 0))
        self.timer9 = QTimer()
        self.timer9.timeout.connect(lambda: self.rosnode.axisButtonHandler(5, True))
        self.timer10 = QTimer()
        self.timer10.timeout.connect(lambda: self.rosnode.axisButtonHandler(5, 0))
        self.timer11 = QTimer()
        self.timer11.timeout.connect(lambda: self.rosnode.axisButtonHandler(6, True))
        self.timer12 = QTimer()
        self.timer12.timeout.connect(lambda: self.rosnode.axisButtonHandler(6, 0))
        self.timers_list = [self.timer1, self.timer2, self.timer3, self.timer4, self.timer5, self.timer6, self.timer7, self.timer8, self.timer9, self.timer10, self.timer11, self.timer12]

    def start_timer_handler(self, timer):
        timer.start(50)

    def stop_timer_handler(self, timer):
        timer.stop()


    def setup(self, rosnode):
        self.rosnode = rosnode

        # Add button click events - There are up and down buttons for each joint
        self.axis1UpButton.pressed.connect(lambda: self.start_timer_handler(self.timer1))
        self.axis1UpButton.released.connect(lambda: self.stop_timer_handler(self.timer1))
        self.axis1DownButton.pressed.connect(lambda: self.start_timer_handler(self.timer2))
        self.axis1DownButton.released.connect(lambda: self.stop_timer_handler(self.timer2))
        self.axis2UpButton.pressed.connect(lambda: self.start_timer_handler(self.timer3))
        self.axis2UpButton.released.connect(lambda: self.stop_timer_handler(self.timer3))
        self.axis2DownButton.pressed.connect(lambda: self.start_timer_handler(self.timer4))
        self.axis2DownButton.released.connect(lambda: self.stop_timer_handler(self.timer4))
        self.axis3UpButton.pressed.connect(lambda: self.start_timer_handler(self.timer5))
        self.axis3UpButton.released.connect(lambda: self.stop_timer_handler(self.timer5))
        self.axis3DownButton.pressed.connect(lambda: self.start_timer_handler(self.timer6))
        self.axis3DownButton.released.connect(lambda: self.stop_timer_handler(self.timer6))
        self.axis4UpButton.pressed.connect(lambda: self.start_timer_handler(self.timer7))
        self.axis4UpButton.released.connect(lambda: self.stop_timer_handler(self.timer7))
        self.axis4DownButton.pressed.connect(lambda: self.start_timer_handler(self.timer8))
        self.axis4DownButton.released.connect(lambda: self.stop_timer_handler(self.timer8))
        self.axis5UpButton.pressed.connect(lambda: self.start_timer_handler(self.timer9))
        self.axis5UpButton.released.connect(lambda: self.stop_timer_handler(self.timer9))
        self.axis5DownButton.pressed.connect(lambda: self.start_timer_handler(self.timer10))
        self.axis5DownButton.released.connect(lambda: self.stop_timer_handler(self.timer10))
        self.axis6UpButton.pressed.connect(lambda: self.start_timer_handler(self.timer11))
        self.axis6UpButton.released.connect(lambda: self.stop_timer_handler(self.timer11))
        self.axis6DownButton.pressed.connect(lambda: self.start_timer_handler(self.timer12))
        self.axis6DownButton.released.connect(lambda: self.stop_timer_handler(self.timer12))



        # Zero button events
        self.zeroAxis1Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(1))
        self.zeroAxis2Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(2))
        self.zeroAxis3Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(3))
        self.zeroAxis4Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(4))
        self.zeroAxis5Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(5))
        self.zeroAxis6Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(6))

        # Gripper button
        self.gripperButton.clicked.connect(lambda: self.rosnode.gripperButtonHandler())




class InterfaceNode(Node):
    def __init__(self, interface):
        super().__init__('interface_node')
        self.get_logger().info('Interface node started')
        self.interface = interface
        self.get_logger().info('Interface GUI started')
        self.gripper_state = False

        # Add subscription and publishers
        self.joint_positions_subscription = self.create_subscription(
            JointState,
            '/workstation2/mover6_joint_states',
            self.joint_positions_callback,
            10
        )

        self.joint_control_publisher = self.create_publisher(
            Mover6Control,
            '/workstation2/mover6_control',
            10
        )
        self.joint_states = JointState()
        self.joint_states.name = ['Joint0', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5']
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def zeroAxisButtonHandler(self, joint_no):
        msg = Mover6Control()
        msg.command = "ZERO"
        msg.joint_no = int(joint_no)
        self.joint_states.position[joint_no-1] = 0 
        msg.joint_angles = [float(a) for a in self.joint_states.position]
        self.joint_control_publisher.publish(msg)

    
    def axisButtonHandler(self, joint_no, direction):
        # Jog the joint in the given direction
        msg = Mover6Control()
        joint_angles = self.joint_states.position # Degrees
        angle = float(joint_angles[joint_no-1])
        if direction == True: # True is up
            angle = joint_angles[joint_no-1]
            angle += 1
        else:
            angle = joint_angles[joint_no-1]
            angle -= 1

        # The control message should be in degrees, the feedback message is in radians
        msg.joint_angles = [float(a) for a in joint_angles]
        msg.joint_angles[joint_no-1] = angle # This is in degrees
        self.joint_states.position = [float(a) for a in msg.joint_angles] # Local joint states is in degrees
        self.joint_control_publisher.publish(msg)

    def gripperButtonHandler(self):
        self.gripper_state = not self.gripper_state # toggle the gripper state
        # Open or close the gripper
        msg = Mover6Control()
        if self.gripper_state:
            msg.command = "GRIPPER OPEN"
        else:
            msg.command = "GRIPPER CLOSED"
        self.joint_control_publisher.publish(msg)

    def joint_positions_callback(self, msg):
        # self.joint_states = msg
        RAD_TO_DEG = 180/np.pi
        angles = [float(a*RAD_TO_DEG) for a in msg.position]
        negate_list = [-1, -1, 1, -1, 1, 1]
        angles = [a*n for a, n in zip(angles, negate_list)] # Conversion for RVIZ
        self.joint_states.position = angles # Local joint states is in degrees

        
        # self.joint_angles = angles
        self.interface.axis1PositionLabel.setText(str(angles[0]))
        self.interface.axis2PositionLabel.setText(str(angles[1]))
        self.interface.axis3PositionLabel.setText(str(angles[2]))
        self.interface.axis4PositionLabel.setText(str(angles[3]))
        self.interface.axis5PositionLabel.setText(str(angles[4]))
        self.interface.axis6PositionLabel.setText(str(angles[5]))




def main(args=None):
    rclpy.init(args=args)

    # Run the interface and the ros node using multithreaded executor
    app = QApplication(sys.argv)
    gui = Interface()

    # Create the interface node
    node = InterfaceNode(gui)

    # Setup the interface with the ros node
    gui.setup(node)

    # Create a multithreaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Start the ROS2 node in a seperate thread
    thread = Thread(target=executor.spin)
    thread.start()
    node.get_logger().info('ROS2 node started')

    # App running in main thread
    try: 
        gui.show()
        sys.exit(app.exec_())
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

