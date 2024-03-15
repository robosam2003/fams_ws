# An example interface node for the gui

import rclpy
from rclpy.node import Node
import Controller_ui as Controller_ui
import sys
from cv_bridge import CvBridge
from threading import Thread 
import cv2 as cv

from PySide6.QtWidgets import QMainWindow, QApplication
from rclpy.executors import MultiThreadedExecutor

from fams_interfaces.msg import Mover6Control


# Interface class for the GUI
class Interface(QMainWindow, Controller_ui.Ui_MainWindow):
    def __init__(self, parent=None):
        super(Interface, self).__init__(parent)
        self.setupUi(self)
        self.rosnode = None

    def setup(self, rosnode):
        self.rosnode = rosnode

        # Add button click events - There are up and down buttons for each joint
        self.axis1UpButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(1, True))
        self.axis1DownButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(1, 0))
        self.axis2UpButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(2, True))
        self.axis2DownButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(2, 0))
        self.axis3UpButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(3, True))
        self.axis3DownButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(3, 0))
        self.axis4UpButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(4, True))
        self.axis4DownButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(4, 0))
        self.axis5UpButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(5, True))
        self.axis5DownButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(5, 0))
        self.axis6UpButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(6, True))
        self.axis6DownButton.clicked.connect(lambda: self.rosnode.axisButtonHandler(6, 0))

        # Zero button events
        self.zeroAxis1Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(1))
        self.zeroAxis2Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(2))
        self.zeroAxis3Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(3))
        self.zeroAxis4Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(4))
        self.zeroAxis5Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(5))
        self.zeroAxis6Button.clicked.connect(lambda: self.rosnode.zeroAxisButtonHandler(6))




class InterfaceNode(Node):
    def __init__(self, interface):
        super().__init__('interface_node')
        self.get_logger().info('Interface node started')
        self.interface = interface
        self.get_logger().info('Interface GUI started')

        # Add subscription and publishers
        self.joint_positions_subscription = self.create_subscription(
            Mover6Control,
            'mover6_state',
            self.joint_positions_callback,
            10
        )

        self.joint_positions_publisher = self.create_publisher(
            Mover6Control,
            'mover6_control',
            10
        )
        self.joint_angles = [0, 0, 0, 0, 0, 0]

    def zeroAxisButtonHandler(self, joint_no):
        msg = Mover6Control()
        msg.command = "ZERO"
        msg.joint_no = int(joint_no)
        self.joint_angles[joint_no-1] = 0# if joint_no < 5 else 32000
        msg.joint_angles = [float(a) for a in self.joint_angles]
        self.joint_positions_publisher.publish(msg)

    
    def axisButtonHandler(self, joint_no, direction):
        # Jog the joint in the given direction
        msg = Mover6Control()
        angle = float(self.joint_angles[joint_no-1])
        if direction == True: # True is up
            angle = self.joint_angles[joint_no-1]
            angle += 5
        else:
            angle = self.joint_angles[joint_no-1] # 5 degrees jog
            angle -= 5

        
        msg.joint_angles = [float(a) for a in self.joint_angles]
        msg.joint_angles[joint_no-1] = angle
        self.joint_angles = msg.joint_angles
        self.joint_positions_publisher.publish(msg)


    def joint_positions_callback(self, msg):
        angles = msg.joint_angles
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

