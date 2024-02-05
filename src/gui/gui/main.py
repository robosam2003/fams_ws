import rclpy
from rclpy.node import Node

import Interface_ui as ui

# import main windows and qt stuff
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt


class MainWindow(QMainWindow, ui.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('Scheduler')
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.show()


class GUI(Node):
    def __init__(self):
        super().__init__('gui')
        self.get_logger().info('GUI node has been started')

        self.app = QApplication([])
        self.window = MainWindow()
        self.app.exec_()


def main(args=None):
    rclpy.init(args=args)
    gui = GUI()
    rclpy.spin(gui)
    gui.destroy_node()
    rclpy.shutdown()



