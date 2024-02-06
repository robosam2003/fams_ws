import rclpy
from rclpy.node import Node
import sys

import gui.Interface_ui as ui
import time
import random
from fams_interfaces.msg import JobMessage, SubProcess

# import main windows and qt stuff
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6 import QtCore, QtWidgets, QtGui


class Interface(QMainWindow, ui.Ui_MainWindow):
    def __init__(self, rosnode, parent=None):
        super(Interface, self).__init__(parent)
        self.setupUi(self)
        self.rosnode = rosnode
        self.pushButton.clicked.connect(self.rosnode.start)

        self.destroyed.connect(self.rosnode.destroy_node)


class InterfaceNode(Node):
    def __init__(self):
        super().__init__('gui')
        self.get_logger().info('GUI node has been started')
        self.job_publisher = self.create_publisher(
            JobMessage,
            'job',
            10
        )

        self.interface = Interface(self)
        self.interface.show()

        # When the window is closed, destroy the node and stop the application

    def start(self):
        self.interface.label.setText('Generating and publishing a job message...')

        msg = JobMessage()
        msg.job_id = random.randint(1, 100000)
        msg.priority = 1
        msg.part_id = random.randint(1, 100)

        sub1 = SubProcess()
        sub1.sub_process_id = random.randint(1, 100)
        sub1.operation_type = 'milling'
        sub2 = SubProcess()
        sub2.sub_process_id = random.randint(1, 100)
        sub2.operation_type = 'drilling'
        msg.subprocesses = [sub1, sub2]

        msg.subprocess_start_times = [int(time.time()), int(time.time()) + 10]
        msg.subprocess_end_times = [int(time.time()) + 5, int(time.time()) + 15]
        msg.status = 'PENDING'

        # Publish the message
        self.job_publisher.publish(msg)
        self.get_logger().info('Job message has been published')
        self.interface.label.setText('Job message has been published')




def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    node = InterfaceNode()
    app.exec_()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



