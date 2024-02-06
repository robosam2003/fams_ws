import rclpy
from rclpy.node import Node
import sys

import gui.Interface_ui as ui

# import main windows and qt stuff
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6 import QtCore, QtWidgets, QtGui


class Interface(QMainWindow, ui.Ui_MainWindow):
    def __init__(self, rosnode, parent=None):
        super(Interface, self).__init__(parent)
        self.setupUi(self)
        self.rosnode = rosnode
        self.pushButton.clicked.connect(self.rosnode.start)
        self.pushButton_2.clicked.connect(self.rosnode.stop)

         

    


class InterfaceNode(Node):
    def __init__(self):
        super().__init__('gui')
        self.get_logger().info('GUI node has been started')

        self.interface = Interface(self)
        self.interface.show()

    def start(self):
        self.get_logger().info('Start button has been pressed')
        self.interface.label.setText('Start button has been pressed')

    def stop(self):
        self.get_logger().info('Stop button has been pressed')
        self.interface.label.setText('Stop button has been pressed')


def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    node = InterfaceNode()
    app.exec_()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



