# An example interface node for the gui

import rclpy
from rclpy.node import Node
import Interface_ui as Interface_ui
import sys
from cv_bridge import CvBridge
from threading import Thread 
import cv2 as cv

from PySide6.QtWidgets import QMainWindow, QApplication
from rclpy.executors import MultiThreadedExecutor


# Interface class for the GUI
class Interface(QMainWindow, Interface_ui.Ui_MainWindow):
    def __init__(self, parent=None):
        super(Interface, self).__init__(parent)
        self.setupUi(self)
        self.rosnode = None

    def setup(self, rosnode):
        self.rosnode = rosnode
        self.start_button.clicked.connect(self.rosnode.start)
        self.stop_button.clicked.connect(self.rosnode.stop)
        self.pause_button.clicked.connect(self.rosnode.pause)

class InterfaceNode(Node):
    def __init__(self, interface):
        super().__init__('interface_node')
        self.get_logger().info('Interface node started')
        self.interface = interface
        self.bridge = CvBridge()
        self.get_logger().info('Interface GUI started')

        # Add subscription and publishers


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

