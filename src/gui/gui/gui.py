import rclpy
from rclpy.node import Node
import sys

import gui.Interface_ui as ui
import time
import random
import numpy as np
from fams_interfaces.msg import Job, JobList, SubProcess, Part, SystemState, EmergencyControl, LoadUnload

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
        self.partObj=Part()
        self.partIdCheck = 0
        self.addToListCheck = 0
        self.preset_path = "./src/gui/gui/preset.csv"  # Relative so that it works on any machine
        self.Layout1Toggle = 0
        self.Layout2Toggle = 0
        #self.stopButton.clicked.connect(self.rosnode.start) # if stopButton is clicked
        
        # jobListWidget testing vvv
        # self.jobListWidget.setSelectionMode(QAbstractItemView.Mul)
        #self.jobListWidget.addItem("One")
        #self.jobListWidget.addItems(["Two","Three"])
        
        #self.jobListWidget.setCurrentRow(1)
        #print(self.jobListWidget.currentRow)
        #self.PartIDList = [1908622204159,2228622204159,3338622204159;"Block1","Block2","Block3"]
        self.PartIDList = [[19085255111214,"Yellow Block"],[19086421029,"Blue BLock"],[1908621216136,"Green Block"],[1908519983210,"Red Block"],[1908622204159,"Orange Block"]]
        for i in range(len(self.PartIDList)):
            self.subprocessListWidget_3.addItem(str(self.PartIDList[i][0]))
            self.subprocessListWidget_4.addItem(str(self.PartIDList[i][1]))

        self.jobObj=Job() # creates an object of class Job
        self.jobObj.subprocesses=[] # creates a blank array to store a list of subprocesses

        self.emergency = EmergencyControl()

        self.clearSubprocessesList()

        self.destroyed.connect(self.rosnode.destroy_node)

        self.stopButton.clicked.connect(self.stopButtonHandler)
        self.addToList.clicked.connect(self.addToListHandler)
        self.addJobButton.clicked.connect(self.addJobButtonHandler)#add job btn

        self.jobListWidget.clicked.connect(self.jobListWidgetHandler) # if jobListWidget is clicked
        #self.jobListWidget.selectedItems()
        self.jobListWidget.currentItemChanged.connect(self.jobListWidget_current_item_changed) # if the selected item has changed
        self.jobListWidget.currentTextChanged.connect(self.current_text_changed) # if the selected text has changed
        self.subprocessListWidget_3.currentItemChanged.connect(self.subprocessListWidget_3_current_item_changed) # if the selected item has changed

        self.subprocessListWidget_1.currentRowChanged.connect(self.subprocess1_row_changed) # if the selected item has changed 
        self.subprocessListWidget_2.currentRowChanged.connect(self.subprocess2_row_changed) # if the selected item has changed 
        self.subprocessListWidget_3.currentRowChanged.connect(self.part1_row_changed) # if the selected item has changed (PART LIST 1)
        self.subprocessListWidget_4.currentRowChanged.connect(self.part2_row_changed) # if the selected item has changed (PART LIST 2)

        self.JobDeleteButton.clicked.connect(self.JobDeleteButtonHandler)
        self.ClearSubprocessesButton.clicked.connect(self.ClearSubprocessesButtonHandler)
        self.PartSelectButton.clicked.connect(self.PartSelectButtonHandler)
        self.LoadedButton.clicked.connect(self.LoadedButtonhandler)
        self.UnloadedButton.clicked.connect(self.UnloadedButtonHandler)
        self.LoadPresetButton.clicked.connect(self.LoadPresetButtonHandler)
        self.LayoutButton1.clicked.connect(self.LayoutButton1handler)
        self.LayoutButton2.clicked.connect(self.LayoutButton2handler)

    def LayoutButton1handler(self):
        if self.Layout1Toggle == 0:
            self.Layout1Toggle = 1
            self.stopButton.setStyleSheet("background-color: rgb(255, 64, 0)")
            self.frame1.setStyleSheet("background-color: rgb(96, 129, 99)")
            self.frame.setStyleSheet("background-color: rgb(131, 175, 134)")
            self.addToList.setStyleSheet("background-color: rgb(139, 185, 142)")
        else:
            self.Layout1Toggle = 0
            self.stopButton.setStyleSheet("background-color: rgb(255, 0, 0)")
            self.frame1.setStyleSheet("background-color: rgb(100, 100, 120)")
            self.frame.setStyleSheet("background-color: rgb(125, 155, 200)")
            self.addToList.setStyleSheet("background-color: rgb(70, 100, 200)")

    def LayoutButton2handler(self):
        self.csv_to_lineEdits()

    def LoadPresetButtonHandler(self):
        self.csv_to_lineEdits()
    
    def LoadedButtonhandler(self):
        selected_part_id = self.partWidgetItem.text()
        if selected_part_id == None:
            self.rosnode.get_logger().warn('No part selected')
            return
        # system_state = self.rosnode.system_state
        # parts = system_state.parts
        # for part in parts:
        #     if part.part_id == selected_part_id:
        #         # part is the chosen part
        #         # If the current subprocess is 0, then the part needs moving, double check that the part is "NEW" (has no previous subprocess)
        #         if part.current_subprocess_id == 0 and part.previous_subprocess_id == 0: 
        #             # The button is clicked, so the part is loaded onto the robot
        #             part.previous_subprocess_id = 1 # 1 is the id for loading                    
        #             # Scheduler will handle the rest
        
        # # Publish the updated system state
        # self.rosnode.system_state_publisher.publish(system_state)
        load_unload_msg = LoadUnload()
        load_unload_msg.part_id = selected_part_id
        load_unload_msg.command = "LOAD"
        self.rosnode.load_unload_publisher.publish(load_unload_msg)

    def UnloadedButtonHandler(self):
        selected_part_id = self.partWidgetItem.text()
        if selected_part_id == None:
            self.rosnode.get_logger().warn('No part selected')
            return
        # system_state = self.rosnode.system_state
        # parts = system_state.parts
        # for part in parts:
        #     if part.part_id == selected_part_id:
        #         # part is the chosen part
        #         # If the current subprocess is 0, then the part needs moving, double check that the part is not "new" (has a previous subprocess)
        #         if part.current_subprocess_id == 0 and parts.previous_subprocess_id != 0: 
        #             # The button is clicked, so the part is loaded onto the robot
        #             part.previous_subprocess_id = -1 # -1 is the id for unloading    
        #             part.next_subprocess_id = 0 # i.e. the part is finished, it has been unloaded - Scheduler will handle the removal of the part from the system                
        #             # Scheduler will handle the rest
        # # Publish the updated system state
        # self.rosnode.system_state_publisher.publish(system_state)
        load_unload_msg = LoadUnload()
        load_unload_msg.part_id = selected_part_id
        load_unload_msg.command = "UNLOAD"
        self.rosnode.load_unload_publisher.publish(load_unload_msg)
        
    def jobListWidgetHandler(self, text):
        #self.jobListWidget.takeItem(self.jobListWidget.currentRow())
        #self.label_10.setText(str(self.jobListWidget.selectedItems()))
        #print("Current item : ",item.text())
        #print("Current text changed : ",text)
        print("jobListWidgetHandler")

    def subprocess1_row_changed(self, row):
        print("Current row : ",row)
        self.subprocessListWidget_2.setCurrentRow(row)

    def subprocess2_row_changed(self, row):
        print("Current row : ",row)
        self.subprocessListWidget_1.setCurrentRow(row)

    def part1_row_changed(self, row):
        print("Current row : ",row)
        self.subprocessListWidget_4.setCurrentRow(row)

    def part2_row_changed(self, row):
        print("Current row : ",row)
        self.subprocessListWidget_3.setCurrentRow(row)

    def jobListWidget_current_item_changed(self, item):
        print("Current item : ",item.text())
        self.jobWidgetItem = item

    def subprocessListWidget_3_current_item_changed(self, item):
        print("Current item : ",item.text())
        self.partWidgetItem = item

    def current_text_changed(self,text):
        print("Current text changed : ",text)
        #self.label_10.setText(str("item.text", text))
    
    def stopButtonHandler(self):
        print("stop")
        self.stopButton.setStyleSheet("background-color: rgb(143, 0, 0)")
        self.widget.setStyleSheet("background-color: rgb(200, 155, 155)")
        #self.emergency = 1
        #self.rosnode.emergency_publisher.publish(self.emergency)
       # self.get_logger().info('stop')

        #print(str(self.rosnode.jobList[0].job_id))
        
    def JobDeleteButtonHandler(self):
        #print('currentRow: ',str(self.jobListWidget.currentRow()))
        #self.jobListWidget.takeItem(self.jobListWidget.currentRow())

        #self.rosnode.get_logger().info(str(self.jobListWidget.currentRow()))
        print('currentRow: ',str(self.jobListWidget.currentRow()))

        print('currentItem***: ',str(self.jobWidgetItem.text()))
        self.jobObj.job_id = int(self.jobWidgetItem.text())
        #self.jobObj.job_id = 11111
        self.jobObj.priority=0
        partObj=Part()
        partObj.part_id = '0'
        partObj.location = '0'
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

    def PartSelectButtonHandler(self):
        print('currentRow: ',str(self.subprocessListWidget_3.currentRow()))
        print('currentItem***: ',str(self.partWidgetItem.text()))
        self.partObj.part_id = str(self.partWidgetItem.text())
        self.partIdCheck = 1
        self.AddJobButtonCheck()

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
        self.jobObj.subprocesses.append(sub1) # <-- appends to subprocess list here
        print((self.jobObj.subprocesses[0]))
        
        self.addToListCheck=1
        self.AddJobButtonCheck()

        self.subprocessListWidgetUpdater()
        #else:
            #self.addToList.setStyleSheet("background-color: rgb(50, 71, 143)")
            #self.addToList.setText("All fields must be filled correctly.")
        
        
    def subprocessListWidgetUpdater(self):
        self.subprocessListWidget_1.clear() # clears subprocess_id listWidget
        self.subprocessListWidget_2.clear() # clears subprocess operation listWidget
        for i in range(len(self.jobObj.subprocesses)):
            self.subprocessListWidget_1.addItem(str(self.jobObj.subprocesses[i].sub_process_id))
            self.subprocessListWidget_2.addItem(str(self.jobObj.subprocesses[i].operation_type))
            print(str(i))
            print('subList[i].sub_id = ', str(self.jobObj.subprocesses[i].sub_process_id))
#
    def addJobButtonHandler(self):
        self.jobObj.job_id=int(self.job_id.text())

        
        #partObj.part_id = int(self.part_id.text()) # part id no longer comes from lineEdit
        self.partObj.part_id = str(self.partWidgetItem.text())
        self.partObj.location = str(self.partLocation.text())
        self.partObj.current_subprocess_id = 0
        self.partObj.job_id = self.jobObj.job_id
        self.jobObj.part = self.partObj

        self.jobObj.start_time=int(self.startTime.text())
        self.jobObj.end_time=int(self.endTime.text())

        self.jobObj.priority=int(self.priority.currentText())

        self.jobObj.status = 'PENDING'

        #self.jobObj.add_remove = 'ADD'

        print((self.jobObj))

        #self.jobListWidget.addItem(self.jobObj)



        sub1=SubProcess()   #create subprocess object
        sub1.sub_process_id=-1
        sub1.operation_type='UNLOADING'
        sub1.start_time=0
        sub1.end_time=0
        self.jobObj.subprocesses.append(sub1) # <-- appends to subprocess list here
        print("UNLOADING subprocess added")
        self.subprocessListWidgetUpdater()

        self.rosnode.job_publisher.publish(self.jobObj)
        print('Job message has been published with job_id: ', self.jobObj.job_id)
        # self.interface.label.setText('Job message has been published')

        #self.jobObj.subprocesses=[] #clears subprocesses list

    def ClearSubprocessesButtonHandler(self):
        self.clearSubprocessesList() # <-- Subprocess list cleared here
        self.addToListCheck = 0
        self.AddJobButtonCheck()

    def clearSubprocessesList(self):
        self.jobObj.subprocesses=[]
        self.subprocessListWidget_1.clear() # clears subprocess_id listWidget
        self.subprocessListWidget_2.clear() # clears subprocess operation listWidget
        sub1=SubProcess()   #create subprocess object
        sub1.sub_process_id=1
        sub1.operation_type='LOADING'
        sub1.start_time=0
        sub1.end_time=0
        self.jobObj.subprocesses.append(sub1) # <-- appends to subprocess list here
        print("LOADING subprocess added")
        self.subprocessListWidgetUpdater()

    def AddJobButtonCheck(self):
        if self.partIdCheck == 1 and self.addToListCheck == 1:
            self.addJobButton.setStyleSheet("background-color: rgb(143, 143, 191)") # <-- addJobButton turns green here
        else:
            self.addJobButton.setStyleSheet("background-color: rgb(60, 60, 80)") # <-- addJobButton turns back to original colour here

    def csv_to_lineEdits(self):
        # Convert csv string to Job message

        with open(self.preset_path, "r") as f:
            for line in f:
                csv_list = line.split(',')
                job = Job()
                self.rosnode.get_logger().info(csv_list[0])
                #self.rosnode.get_logger().info(int(csv_list[0]))
                job.job_id = int(csv_list[0])
                self.job_id.setText(csv_list[0])
                #job.priority = int(csv_list[1])

                part = Part()
                #part.part_id = str(csv_list[2])
                part.location = str(csv_list[3])
                self.partLocation.setText(csv_list[3])
                #part.current_subprocess_id = int(csv_list[4])
                #part.job_id = int(csv_list[5])
                #job.part = part

                part_obj_size = 6 # Number of parameters in the Part object
                sub_obj_size = 4 # Number of parameters in the SubProcess object
                sub_start_id = 2 + part_obj_size
                n_sub = (len(csv_list) - part_obj_size - 5) // sub_obj_size
                for i in range(n_sub):
                    sub = SubProcess()
                    sub.sub_process_id = int(csv_list[sub_start_id + sub_obj_size*i])
                    sub.operation_type = str(csv_list[sub_start_id + sub_obj_size*i + 1])
                    sub.start_time = int(csv_list[sub_start_id + sub_obj_size*i + 2])
                    sub.end_time = int(csv_list[sub_start_id + sub_obj_size*i + 3])
                    job.subprocesses.append(sub)
                    print(" i =  ",i)
                    if i == 1:
                        self.subID.setText(csv_list[sub_start_id + sub_obj_size*i])
                        self.opType.setText(csv_list[sub_start_id + sub_obj_size*i + 1])
                        self.subStartTime.setText(csv_list[sub_start_id + sub_obj_size*i + 2])
                        self.subEndTime.setText(csv_list[sub_start_id + sub_obj_size*i + 3])

        
                #print('csv_list[-3] = ', csv_list[-3])
                #print('csv_list[-2] = ', csv_list[-2])
                #print('csv_list[-1] = ', csv_list[-1])
                job.start_time = int(csv_list[-3])
                self.startTime.setText(csv_list[-3])
                job.end_time = int(csv_list[-2])
                self.endTime.setText(csv_list[-2])
                job.status = str(csv_list[-1])
        



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
        # self.subscription = self.create_subscription(
        #     CompressedImage, 
        #     'video_stream', 
        #     self.listener_callback, 
        #     10
        # )
        # self.subscription # prevent unused variable warning

        self.active_jobs_subscription = self.create_subscription(
            JobList,
            'active_jobs',
            self.jobList_callback,
            10
        )

        self.system_state_subscription = self.create_subscription(
            SystemState,
            '/system_state',
            self.system_state_callback,
            10
        )

        # self.system_state_publisher = self.create_publisher(
        #     SystemState,
        #     '/system_state',
        #     10
        # )

        self.load_unload_publisher = self.create_publisher(
            LoadUnload,
            'load_unload',
            10
        )
      
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.system_state = SystemState()
        
        self.interface = Interface(self)
        self.interface.show()

    def system_state_callback(self, msg: SystemState):
        self.system_state = msg

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

        self.interface.jobListWidget.clear()
        
        # for loop to add jobs from jobList into gui

        #print(self.jobList[0])
        
        #self.interface.jobListWidget.addItems(["Two","Three"])
        print(str(len(self.jobList)))

        for i in range(len(self.jobList)):
            self.interface.jobListWidget.addItem(str(self.jobList[i].job_id)) #******* NEED TO REMOVE ITEMS FROM WIDGET FIRST*********
            print(str(i))
            print('jobList[i].job_id = ', str(self.jobList[i].job_id))

        #self.interface.jobListWidget.addItem(str(self.jobList[0].job_id))
        

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



