import rclpy
from rclpy.node import Node
import sys

import gui.Interface_ui as ui
import time
import random
from fams_interfaces.msg import Job, SubProcess, Part, SystemState

from threading import Thread
from rclpy.executors import MultiThreadedExecutor

# import main windows and qt stuff
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6 import QtCore, QtWidgets, QtGui
from std_msgs.msg import String


class Interface(QMainWindow, ui.Ui_MainWindow):
    def __init__(self, rosnode, parent=None):
        super(Interface, self).__init__(parent)
        self.setupUi(self)
        self.rosnode = rosnode
        self.stopButton.clicked.connect(self.rosnode.start)
        #self.stopButton.clicked.print("clicked")


        self.jobObj=Job()
        self.jobObj.subprocesses=[]

        self.stopButton.setStyleSheet("background-color: red")

        self.destroyed.connect(self.rosnode.destroy_node)
        self.stopButton.clicked.connect(self.stopButtonHandler)
        self.addToList.clicked.connect(self.addToListHandler)
        self.addJobButton.clicked.connect(self.addJobButtonHandler)#add job btn
       
    def stopButtonHandler(self):
        print("stop")
       # self.get_logger().info('stop')
    def addToListHandler(self):
        #print(str(self.subID.text()))
        #print(str(self.opType.text()))
               
        
        
        sub1=SubProcess()   #create subprocess object
        sub1.sub_process_id=int(self.subID.text())
        sub1.operation_type=self.opType.text()
        sub1.start_time=int(self.subStartTime.text())
        sub1.end_time=int(self.subEndTime.text())
        print((sub1.sub_process_id))
        print((sub1.operation_type))
        print((sub1.start_time))
        print((sub1.end_time))
        self.jobObj.subprocesses.append(sub1)
        print((self.jobObj.subprocesses[0]))
        
       
        
        #  self.get_logger().info('stop')
#
    def addJobButtonHandler(self):
        self.jobObj.job_id=int(self.job_id.text())

        partObj=Part()
        partObj.part_id = int(self.part_id.text())
        partObj.location = int(self.partLocation.text())
        partObj.current_subprocess_id = 0
        partObj.job_id = self.jobObj.job_id
        self.jobObj.part = partObj

        self.jobObj.start_time=int(self.startTime.text())
        self.jobObj.end_time=int(self.endTime.text())

        self.jobObj.priority=int(self.priority.currentText())

        self.jobObj.status = 'PENDING'

        print((self.jobObj))

        #interfaceNodeObj = InterfaceNode()
        #interfaceNodeObj.job_publisher = interfaceNodeObj.create_publisher(
        #    Job,
        #    'job',
        #    10
        #
        #interfaceNodeObj.job_publisher.publish(self.jobObj
        
        # self.get_logger().info('Job message has been published')
        # self.interface.label.setText('Job message has been published')



class InterfaceNode(Node):
    def __init__(self):
        super().__init__('gui')
        self.get_logger().info('GUI node has been started')
        self.job_publisher = self.create_publisher(
            Job,
            'job',
            10
        )

        self.chatter_subscriber = self.create_subscription(
            String,
            "/chatter", 
            self.string_callback,
            10
        )

        self.interface = Interface(self)
        self.interface.show()

    def string_callback(self, msg: String):
        self.get_logger().info(str(msg))

        

        # When the window is closed, destroy the node and stop the application

    def start(self):
        #self.interface.label.setText('Generating and publishing a job message...')

        #job = Job()
        #job.job_id = random.randint(1, 100000)
        #job.priority = 1
        #part = Part()
        #part.part_id = random.randint(1, 100)
        #part.location = 0
        #part.current_subprocess_id = 0
        #part.job_id = job.job_id
        #job.part = part

        #sub1 = SubProcess()
        #sub1.sub_process_id = random.randint(1, 100)
        #sub1.operation_type = 'milling'
        #sub1.start_time = int(time.time())
        #sub1.end_time = 0
        #sub2 = SubProcess()
        #sub2.sub_process_id = random.randint(1, 100)
        #sub2.operation_type = 'drilling'
        #sub2.start_time = int(time.time()) + 50
        #sub2.end_time = 0
        #job.subprocesses = [sub1, sub2]
        #job.start_time = sub1.start_time
        #job.end_time = 0
        
        #job.status = 'PENDING'

        # Publish the message
        self.job_publisher.publish(self.interface.jobObj)
        # self.get_logger().info('Job message has been published')
        # self.interface.label.setText('Job message has been published')



def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    node = InterfaceNode()
    

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.get_logger().info(str(executor.get_nodes()))
    # Start the ROS2 node on a separate thread
    thread = Thread(target=executor.spin)
    thread.start()
    node.get_logger().info("Spinning ROS2 node . . .")
    # rclpy.spin(node)
    app.exec_()

    node.get_logger().info("Shutting down ROS2 node . . . ")
    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()



