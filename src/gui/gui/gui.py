import rclpy
from rclpy.node import Node
import sys

import gui.Interface_ui as ui
import time
import random
from fams_interfaces.msg import Job, JobList, SubProcess, Part, SystemState, EmergencyControl

from threading import Thread
from rclpy.executors import MultiThreadedExecutor

# import main windows and qt stuff
from PySide6.QtWidgets import QWidget, QMainWindow, QApplication, QListView, QListWidget, QAbstractItemView, QTableView, QCheckBox, QHBoxLayout, QVBoxLayout, QPushButton, QButtonGroup
from PySide6 import QtCore, QtWidgets, QtGui, QtQuick
from std_msgs.msg import String

from sensor_msgs.msg import Image, CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

class Interface(QMainWindow, ui.Ui_MainWindow, QWidget):
    def __init__(self, rosnode, parent=None):
        super(Interface, self).__init__(parent)
        self.setupUi(self)
        self.rosnode = rosnode
        
        #self.stopButton.clicked.connect(self.rosnode.start) # if stopButton is clicked
        
        # listWidget testing vvv
        # self.listWidget.setSelectionMode(QAbstractItemView.Mul)
        #self.listWidget.addItem("One")
        #self.listWidget.addItems(["Two","Three"])
        
        #self.listWidget.setCurrentRow(1)
        #print(self.listWidget.currentRow)

        self.jobObj=Job() # creates an object of class Job
        self.jobObj.subprocesses=[] # creates a blank array to store a list of subprocesses

        self.emergency = EmergencyControl()

        self.destroyed.connect(self.rosnode.destroy_node)

        self.stopButton.clicked.connect(self.stopButtonHandler)
        self.addToList.clicked.connect(self.addToListHandler)
        self.addJobButton.clicked.connect(self.addJobButtonHandler)#add job btn

        self.listWidget.clicked.connect(self.listWidgetHandler) # if listWidget is clicked
        #self.listWidget.selectedItems()
        self.listWidget.currentItemChanged.connect(self.current_item_changed) # if the selected item has changed
        self.listWidget.currentTextChanged.connect(self.current_text_changed) # if the selected text has changed

        self.JobDeleteButton.clicked.connect(self.JobDeleteButtonHandler)
       
    def listWidgetHandler(self, text):
        #self.listWidget.takeItem(self.listWidget.currentRow())
        #self.label_10.setText(str(self.listWidget.selectedItems()))
        #print("Current item : ",item.text())
        #print("Current text changed : ",text)
        print("listWidgetHandler")

    def current_item_changed(self, item):
        print("Current item : ",item.text())
        self.widgetItem = item

    def current_text_changed(self,text):
        print("Current text changed : ",text)
        #self.label_10.setText(str("item.text", text))
    
    def stopButtonHandler(self):
        print("stop")
        #self.emergency = 1
        #self.rosnode.emergency_publisher.publish(self.emergency)
       # self.get_logger().info('stop')

        #print(str(self.rosnode.jobList[0].job_id))
        
    def JobDeleteButtonHandler(self):
        #print('currentRow: ',str(self.listWidget.currentRow()))
        #self.listWidget.takeItem(self.listWidget.currentRow())

        #self.rosnode.get_logger().info(str(self.listWidget.currentRow()))
        print('currentRow: ',str(self.listWidget.currentRow()))

        print('currentItem***: ',str(self.widgetItem.text()))
        self.jobObj.job_id = int(self.widgetItem.text())
        #self.jobObj.job_id = 11111
        self.jobObj.priority=0
        partObj=Part()
        partObj.part_id = 0
        partObj.location = 0
        partObj.current_subprocess_id = 0
        partObj.job_id = self.jobObj.job_id
        self.jobObj.part = partObj
        sub1=SubProcess()   #create subprocess object
        sub1.sub_process_id=0
        sub1.operation_type=''
        sub1.start_time=0
        sub1.end_time=0
        self.jobObj.subprocesses.append(sub1)
        self.jobObj.start_time = 0
        self.jobObj.end_time = 0
        self.jobObj.status = 'REMOVED'
        #self.jobObj.add_remove = 'REMOVE'
        self.rosnode.job_publisher.publish(self.jobObj)

    def addToListHandler(self):
        #print(str(self.subID.text()))
        #print(str(self.opType.text()))
               
        #self.listView.create
        print(isinstance(self.subID.text(), int))

        #if isinstance(self.subID.text(), int) and isinstance(self.opType.text(), str) and isinstance(self.subStartTime.text(), (int, float)) and isinstance(self.subEndTime.text(), (int, float)):
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
        
        self.addJobButton.setStyleSheet("background-color: rgb(0, 255, 0)")
        #else:
            #self.addToList.setStyleSheet("background-color: rgb(50, 71, 143)")
            #self.addToList.setText("All fields must be filled correctly.")
        
        
        
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

        #self.jobObj.add_remove = 'ADD'

        print((self.jobObj))

        #self.listWidget.addItem(self.jobObj)

        # interfaceNodeObj = InterfaceNode()
        # interfaceNodeObj.job_publisher = interfaceNodeObj.create_publisher(
        #     Job,
        #     'job',
        #     10
        # )
        # interfaceNodeObj.job_publisher.publish(self.jobObj)
        self.rosnode.job_publisher.publish(self.jobObj)
        print('Job message has been published with job_id: ', self.jobObj.job_id)
        # self.interface.label.setText('Job message has been published')

        self.jobObj.subprocesses=[] #clears subprocesses list

class InterfaceNode(Node):
    def __init__(self):
        super().__init__('gui')
        self.get_logger().info('GUI node has been started')
        self.job_publisher = self.create_publisher(
            Job,
            'job',
            10
        )

        self.emergency_publisher = self.create_publisher(
            EmergencyControl,
            'emergency_control',
            10
        )

        #self.chatter_subscriber = self.create_subscription(
        #    String,
        #    "/chatter", 
        #    self.string_callback,
        #    10
        #)

        #Vid_subscriber
        self.subscription = self.create_subscription(
            CompressedImage, 
            'video_stream', 
            self.listener_callback, 
            10
        )
        self.subscription # prevent unused variable warning

        self.subscription = self.create_subscription(
            JobList,
            'active_jobs',
            self.jobList_callback,
            10
        )
      
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
        # current_frame
        # Display image
        cv2.imshow("camera", current_frame) 
        # look at compressed image types 
        # need a label to write a pixmap to
        cv2.waitKey(1)

    def camera_view_callback(self, msg): # EXAMPLE IMAGE CALLBACK - ADAPT FOR YOUR NEEDS
        self.get_logger().info('Camera view callback')
        # # Convert message to cv image
        converted_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        display_scale = 0.7
        converted_image = cv2.resize(converted_image, fx=display_scale, fy=display_scale, dsize=(0, 0))


        # Scale the image to fit the label
        # # Set the pixmap of the label to the image
        self.interface.vision_label.setPixmap(QtGui.QPixmap.fromImage(QtGui.QImage(converted_image.data, converted_image.shape[1], converted_image.shape[0], QtGui.QImage.Format_BGR888)))


    def string_callback(self, msg: String):
        self.get_logger().info(str(msg))

        

        # When the window is closed, destroy the node and stop the application

    def jobList_callback(self, msg: JobList):
        self.get_logger().info('jobList callback')
        self.jobList = msg.list

        self.interface.listWidget.clear()
        
        # for loop to add jobs from jobList into gui

        #print(self.jobList[0])
        
        #self.interface.listWidget.addItems(["Two","Three"])
        print(str(len(self.jobList)))

        for i in range(len(self.jobList)):
            self.interface.listWidget.addItem(str(self.jobList[i].job_id)) #******* NEED TO REMOVE ITEMS FROM WIDGET FIRST*********
            print(str(i))
            print('jobList[i].job_id = ', str(self.jobList[i].job_id))

        #self.interface.listWidget.addItem(str(self.jobList[0].job_id))
        

    def start(self):
        #self.interface.label.setText('Generating and publishing a job message...')


        #print("hello")
        # Publish the message
        self.job_publisher.publish(self.interface.jobObj)
        self.get_logger().info('Job message has been published')
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



