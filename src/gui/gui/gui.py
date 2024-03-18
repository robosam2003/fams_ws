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

from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

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

        #Vid_subscriber
        self.subscription = self.create_subscription(
            Image, 
            'video_stream', 
            self.listener_callback, 
            10
        )
        self.subscription # prevent unused variable warning
      
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        self.interface = Interface(self)
        self.interface.show()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
 
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        img = QtGui.QImage(current_frame.data)
        current_frame
        # Display image
        cv2.imshow("camera", current_frame)
        
        cv2.waitKey(1)

    def string_callback(self, msg: String):
        self.get_logger().info(str(msg))

        

        # When the window is closed, destroy the node and stop the application

    def start(self):
        #self.interface.label.setText('Generating and publishing a job message...')



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



