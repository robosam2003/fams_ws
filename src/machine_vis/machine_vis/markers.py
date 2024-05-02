#!/usr/bin/env python3
# Author: Ben Harvey
# Node for all mschine vision functionalitiies of th system including ArUco Marker localisation, obstacle detecetion and initialisation
 
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import os
import numpy as np
from fams_interfaces.msg import Vision
from rosidl_runtime_py import *
from geometry_msgs.msg import Point
import math
from statistics import mean


class ArucoReader(Node):
  
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_cdreader')
    self.get_logger().info('Machine Vision Node Started')

    self.declare_parameters(
      namespace='',
      parameters=[
        ('vidIndex', 2)
      ]
    )
    # To change vidIndex at runtime, use:
    #  ros2 run machine_vis marker --ros-args -p vidIndex:=2
    
    self.vidIndex=int(self.get_parameter('vidIndex').value)
    self.get_logger().info(f"VID INDEX: {self.vidIndex}")
     # Before running, it should be verfied that, maincam=0, workBcam=2, workAcam=4

    self.xpart=[]
    self.ypart=[]
    self.yawpart=[]
    match self.vidIndex:
      case 0:
        self.moblocation0_pub = self.create_publisher(Point,'nexus1/aruco_tf',10)
        self.moblocation1_pub = self.create_publisher(Point,'nexus2/aruco_tf',10)
        self.moblocation_pub = self.create_publisher(Vision,'amr_vision_locations',10)
        self.windowname='Main Cam'
      case 2:
        self.partlocation_pub=self.create_publisher(Vision,'workstation2/VisionLocations',10)
        self.windowname='Workstation 2 Cam'
      case 4:
        self.partlocation_pub=self.create_publisher(Vision,'workstation1/VisionLocations',10)
        self.windowname='Workstation 1 Cam'

    self.camera_setup(self.vidIndex)

    aruco_type = "DICT_4X4_100" #Is looking for 4x4 only
    self.armOrigin=[]
    
    self.main_loop(aruco_type)

  def camera_setup(self,CamIndex):
    match CamIndex:
      case 0:
        self.get_logger().info('Setting Up Main Camera...')
        
        self.cap = cv2.VideoCapture(-1,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS,20)
        self.Origin=[-0.63760671, -0.3289283,   9.45661633]
        self.Cam_Mtrx = np.array(((5.14225115e+03, 0.00000000e+00, 1.30071631e+03),(0, 5.21253566e+03,7.22183264e+02),(0,0,1)))
        self.Distort = np.array((2.51186484e-01, -5.65362473e+00,  1.50035516e-02,  1.11397010e-02,1.36994424e+01))
        self.markerSize=0.15
        self.detectObstacles=1
        
        fps=self.cap.get(cv2.CAP_PROP_FPS)
        print(fps)
        width=self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        print(width)
        
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=0')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=125')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=3700')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=gain=30')
        
      case 2:
        self.get_logger().info('Setting Up Workstation 2 Camera...')
        self.cap = cv2.VideoCapture(2,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS,10)
        self.Cam_Mtrx = np.array(((5.14225115e+03, 0.00000000e+00, 1.30071631e+03),(0, 5.21253566e+03,7.22183264e+02),(0,0,1)))
        self.Distort = np.array((2.51186484e-01, -5.65362473e+00,  1.50035516e-02,  1.11397010e-02,1.36994424e+01))
        # self.Cam_Mtrx = np.array(((1.21665111e+03, 0, 6.54768787e+02),(0, 1.21478888e+03, 5.00652432e+02),(0,0,1)))
        # self.Distort = np.array((0.04315798,  0.50036972, -0.01800276, -0.00592732, -1.26928846))
        self.markerSize=0.0325
        self.detectObstacles=0
        
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_automatic=0')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=focus_automatic_continuous=0')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=focus_absolute=0')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=exposure_time_absolute=110')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_temperature=3700')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=gain=30')
        fps=self.cap.get(cv2.CAP_PROP_FPS)
        print(fps)
        width=self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        print(width)
      case 4:
        self.get_logger().info('Setting Up Workstation 1 Camera...')
        self.cap = cv2.VideoCapture(4,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS,10)
        self.Cam_Mtrx = np.array(((5.14225115e+03, 0.00000000e+00, 1.30071631e+03),(0, 5.21253566e+03,7.22183264e+02),(0,0,1)))
        self.Distort = np.array((2.51186484e-01, -5.65362473e+00,  1.50035516e-02,  1.11397010e-02,1.36994424e+01))
        self.markerSize=0.0325
        self.detectObstacles=0
        
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=auto_exposure=1')
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=white_balance_automatic=0')
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=focus_automatic_continuous=0')
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=focus_absolute=0')
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=exposure_time_absolute=130')
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=white_balance_temperature=3700')
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=gain=30')
        fps=self.cap.get(cv2.CAP_PROP_FPS)
        print(fps)
        width=self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        print(width)
    return self.cap
  
  def errorFix_x(self, xval):
    errorfix=(0.146*xval**4+0.04*xval**3-1.11195*xval**2+2.9528*xval)/100
    newx=round(xval-errorfix,2)
    return newx

  def ScreenArucoInfo(self,frame,newx,y,yaw,corners):
    x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
    y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    x_centerPixel = int(x_sum*.25)
    y_centerPixel = int(y_sum*.25)
    msg1="x: "+ str(round(newx,3)) + "m"
    msg2="y: " +str(round(y,3))+"m"
    msg3="yaw: " +str(round(yaw,3))+"rads"
    cv2.putText(frame,msg1,(x_centerPixel-85,y_centerPixel+150),cv2.FONT_HERSHEY_SIMPLEX,0.7, (0, 255, 0),2)
    cv2.putText(frame,msg2,(x_centerPixel-85,y_centerPixel+180),cv2.FONT_HERSHEY_SIMPLEX,0.7, (0, 255, 0),2)
    cv2.putText(frame,msg3,(x_centerPixel-85,y_centerPixel+210),cv2.FONT_HERSHEY_SIMPLEX,0.7, (0, 255, 0),2)

  def convertboxpoints(self,box):
    scale=1/535
    x1 = (box[0][0]-960)*scale
    x1=self.errorFix_x(x1)
    y1 = round((box[0][1]-540)*scale,2)
    x2 = (box[1][0]-960)*scale
    x2=self.errorFix_x(x2)
    y2 = round((box[1][1]-540)*scale,2)
    x3 = (box[2][0]-960)*scale
    x3=self.errorFix_x(x3)
    y3 = round((box[2][1]-540)*scale,2)
    x4 = (box[3][0]-960)*scale
    x4=self.errorFix_x(x4)
    y4 = round((box[3][1]-540)*scale,2)
    p1=(x1,y1)
    p2=(x2,y2)
    p3=(x3,y3)
    p4=(x4,y4)
    return p1,p2,p3,p4
  
  def isRotationMatrix(self,R): # Checks if a matrix is a valid rotation matrix.
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
  def rotationMatrixToEulerAngles(self,R) : # Converts Rotation Matrix to Euler Angles
    assert(self.isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])
  
  def pose_estimation(self,frame,aruco_dict_type,camera_matrix, distortion_vector,markerSize,switch):
    
    # Define Local Function Parameters
    anti_ob_flag=[]
    locations=[]
    loc_msg=[]
    id_msg=[]
    publish=0
    mobilelocation0_msg=Point()
    mobilelocation1_msg=Point()
    vmsg=Vision()
    part_id_msg=[]
    part_loc_msg=[]
    partlocation_msg=Vision()

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Greyscale the Frame
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(grey, cv2.aruco_dict,parameters=parameters,cameraMatrix=camera_matrix,distCoeff=distortion_vector)
    
    if len(corners) > 0:
      for i in range(0,len(ids)):
        if ids[i,0]==82 and switch==0:
          rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerSize, camera_matrix, distortion_vector)
          origin_x=tvec[0][0][0]
          origin_y=tvec[0][0][1]
          origin_z=tvec[0][0][2]
          
          # Transform from 'Origin' Marker to true origin point of the arm
          originmarkerpoint=np.array([origin_x,origin_y,origin_z])
          rmat=cv2.Rodrigues(rvec)[0]
          angles=self.rotationMatrixToEulerAngles(rmat)
          yaw=round((angles[2]),4)
          Rot_to_Arm=np.array(((math.cos(yaw), -math.sin(yaw), 0, 0), 
                               (math.sin(yaw), math.cos(yaw), 0, 0),
                               (0, 0, 1, 0),
                               (0, 0, 0, 1)))
          Trans_to_Arm=np.array(((1, 0, 0, 0),
                               (0, 1, 0, -0.15),
                               (0, 0, 1, 0),
                               (0, 0, 0, 1))) #0.185
          new_origin=np.dot(Rot_to_Arm,Trans_to_Arm)
          originTrans=new_origin[0:3,3]
    
          self.Origin=originmarkerpoint+originTrans # Updates origin when arm origin marker seen
        if ids[i,0]==83 and switch==0:
          rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerSize, camera_matrix, distortion_vector)
          origin_x=tvec[0][0][0]
          origin_y=tvec[0][0][1]
          origin_z=tvec[0][0][2]
          
          # Transform from 'Origin' Marker to true origin point of the arm
          originmarkerpoint=np.array([origin_x,origin_y,origin_z])
          rmat=cv2.Rodrigues(rvec)[0]
          angles=self.rotationMatrixToEulerAngles(rmat)
          yaw=round((angles[2]),4)
          Rot_to_Arm=np.array(((math.cos(yaw), -math.sin(yaw), 0, 0), 
                               (math.sin(yaw), math.cos(yaw), 0, 0),
                               (0, 0, 1, 0),
                               (0, 0, 0, 1)))
          Trans_to_Arm=np.array(((1, 0, 0, -0.155),
                               (0, 1, 0, 0),
                               (0, 0, 1, 0),
                               (0, 0, 0, 1))) #0.185
          new_origin=np.dot(Rot_to_Arm,Trans_to_Arm)
          originTrans=new_origin[0:3,3]
          self.Origin=originmarkerpoint+originTrans # Updates origin when arm origin marker seen
          
      for i in range(0, len(ids)):
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerSize, camera_matrix, distortion_vector)         
        cv2.aruco.drawDetectedMarkers(frame, corners,ids) 
        cv2.aruco.drawAxis(frame, camera_matrix, distortion_vector, rvec, tvec, 0.05)  
        
        locations=tvec-self.Origin
        x=(round(locations[0,0,0],5))
        y=(round(locations[0,0,1],5))
        z=(round(locations[0,0,2],5))

        rmat=cv2.Rodrigues(rvec)[0]
        angles=self.rotationMatrixToEulerAngles(rmat)
        yaw=round((angles[2]),4)

        match self.vidIndex:
          case 0: # For Mobile Robot Detection (Main Cam)
            if ids[i,0]==0:
              
              id_msg.append(int(ids[i,0]))
              errorfix=(0.146*x**4+0.04*x**3-1.11195*x**2+2.9528*x)/100
              newx=round(x-errorfix,5)
              loc_msg.extend([newx,y,yaw])
              anti_ob_flag.append(corners[i])
              
              mobilelocation0_msg.x=newx
              mobilelocation0_msg.y=y
              mobilelocation0_msg.z=yaw
              self.ScreenArucoInfo(frame,newx,y,yaw,corners)
            if ids[i,0]==1:
              # self.ScreenArucoInfo(frame,'Mobile Robot 1',corners)
              id_msg.append(int(ids[i,0]))
              errorfix=(0.146*x**4+0.04*x**3-1.11195*x**2+2.9528*x)/100
              newx=round(x-errorfix,5)
              loc_msg.extend([newx,y,yaw])
              anti_ob_flag.append(corners[i])
              
              mobilelocation1_msg.x=newx
              mobilelocation1_msg.y=y
              mobilelocation1_msg.z=yaw
              self.ScreenArucoInfo(frame,newx,y,yaw,corners)
            vmsg.part_id=id_msg
            vmsg.part_location=loc_msg
          case 2 | 4:
        # For Part Detection (Workstation Cams)
            if ids[i,0] in range(84,89):
              part_id_msg.append(int(ids[i,0]))

              self.xpart.append(x)     # Filling arrays
              self.ypart.append(y)
              self.yawpart.append(yaw)

              if len(self.xpart)==5:   # when array is length 5 take average
                aveX=round(mean(self.xpart),4)
                aveY=round(mean(self.ypart),4)
                aveYaw=round(mean(self.yawpart),4)

                self.xpart=[]          # empty arrays for filling
                self.ypart=[]
                self.yawpart=[]
                publish=1
                if self.vidIndex==2:
                  part_loc_msg.extend([-aveY,aveX,aveYaw])               # Adjusting for frame of arm
                  print('Part Location: {}'.format([-aveY,aveX,aveYaw])) 
                  #print('Part Location: {}'.format([-x,-y,yaw])) 
                elif self.vidIndex==4:
                  part_loc_msg.extend([aveX,-aveY,aveYaw])
                  print('Part Location: {}'.format([aveX,aveY,aveYaw]))
                partlocation_msg.part_id=part_id_msg
                partlocation_msg.part_location=part_loc_msg
              

      if switch==1:  
        self.moblocation0_pub.publish(mobilelocation0_msg)
        self.moblocation1_pub.publish(mobilelocation1_msg)
        self.moblocation_pub.publish(vmsg)
        self.get_logger().info('{}:{}'.format("Publishing Robot 0",mobilelocation0_msg))  
        self.get_logger().info('{}:{}'.format("Publishing Robot 1",mobilelocation1_msg))  
        self.get_logger().info('{}:{}'.format("Publishing",vmsg))  
      if switch==0:
        if publish==1:
          self.partlocation_pub.publish(partlocation_msg)
          self.get_logger().info('{}:{}'.format("Publishing Location for Part IDs",partlocation_msg))
        
      
      # location_msg.obstacle_id=ob_id_msg
      # location_msg.obstacle_location=ob_loc_msg
      # errorfix=(0.146*x**4+0.04*x**3-1.11195*x**2+2.9528*x)/100
      # newx=x-errorfix
      # msg="x: "+ str(x) + "m" +"  y: " +str(y)+"m"+" yaw: " +str(yaw)+"rads" # Code to show location of marker- Keep commented unless for debugging purposes
      #msg2="errorfixer: " + str(errorfix) +"m" + "  Adjusted x: " +str(newx)+"m"
      #cv2.putText(frame, msg,(210,100),cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0))
      #cv2.putText(frame, msg2,(210,150),cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0))
      # cv2.putText(frame, '-y  0 rads',(970,50),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))
      # cv2.putText(frame, '+y  +-pi rads',(970,1000),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))
      # cv2.putText(frame, '-x -pi/2 rads',(20,520),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))
      # cv2.putText(frame, '-x +pi/2 rads',(1800,520),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))
      # cv2.line(frame, (0,540),(1920,540),(255,0,0),2)
      # cv2.line(frame, (953,0),(953,1080),(255,0,0),2)
      
    else:
      locations=0
    return frame, locations, anti_ob_flag

  


  def obstacle_detector(self,frame,anti_ob_flag):
    FilteredContours=[]
    FilteredBoxes=[]
    grey=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    retval,thresh=cv2.threshold(grey,130,255,cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
      area=cv2.contourArea(cnt)
      
      if area > 2500 and area < 12000:
        M=cv2.moments(cnt)
        scale=1/535
        cxi=int(M['m10']/M['m00'])
        cyi=int(M['m01']/M['m00'])
        
        cx = (cxi-960)*scale
        cy = (cyi-540)*scale
        
        if cxi>20 and cyi< 530 and cxi<900:
          cv2.circle(frame,(int(cxi),int(cyi)),5,(255,0,0),-1)
          rect = cv2.minAreaRect(cnt)
          box = cv2.boxPoints(rect)
          box = np.int0(box)
          p1,p2,p3,p4=self.convertboxpoints(box)
          cv2.putText(frame, str(p1),box[0],cv2.FONT_HERSHEY_SIMPLEX,0.6, (255, 255, 255),2)
          cv2.putText(frame, str(p2),box[1],cv2.FONT_HERSHEY_SIMPLEX,0.6, (255, 255, 255),2)
          cv2.putText(frame, str(p3),box[2],cv2.FONT_HERSHEY_SIMPLEX,0.6, (255, 255, 255),2)
          cv2.putText(frame, str(p4),box[3],cv2.FONT_HERSHEY_SIMPLEX,0.6, (255, 255, 255),2)
          
          match len(anti_ob_flag):
            case 1:
              Mark_x1=anti_ob_flag[0][0][0][0]
              Mark_y1=anti_ob_flag[0][0][0][1]
              check=cv2.pointPolygonTest(box,[Mark_x1,Mark_y1],measureDist=False)
              if check==-1:
                FilteredContours.append(cnt)
                FilteredBoxes.append(box)
                cv2.drawContours(frame,[box],0,(0,0,255),1)
            case 2:
              Mark_x1=anti_ob_flag[0][0][0][0]
              Mark_y1=anti_ob_flag[0][0][0][1]
              Mark_x2=anti_ob_flag[1][0][0][0]
              Mark_y2=anti_ob_flag[1][0][0][1]
              check1=cv2.pointPolygonTest(box,[Mark_x1,Mark_y1],measureDist=False)
              check2=cv2.pointPolygonTest(box,[Mark_x2,Mark_y2],measureDist=False)
              if check1==-1 and check2==-1:
                FilteredContours.append(cnt)
                FilteredBoxes.append(box)
                cv2.drawContours(frame,[box],0,(0,0,255),1)
            case 0:
              FilteredContours.append(cnt)
              FilteredBoxes.append(box)
              cv2.drawContours(frame,[box],0,(0,0,255),1)
            case _:
              print("Anti-Obstacle Flag length error")
    cv2.drawContours(frame, contours=FilteredContours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    return FilteredBoxes
    

  def main_loop(self,aruco_type):
    
    while self.cap.isOpened():
      match self.vidIndex:
        case 0:
          os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=130')
          os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=3700')
          os.system('v4l2-ctl -d /dev/video0 --set-ctrl=gain=30')
          os.system('v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=0')
        case 2:
          os.system('v4l2-ctl -d /dev/video2 --set-ctrl=exposure_time_absolute=120')
          os.system('v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_temperature=3700')
          os.system('v4l2-ctl -d /dev/video2 --set-ctrl=gain=30')
          os.system('v4l2-ctl -d /dev/video2 --set-ctrl=focus_absolute=10')
        case 4:
          os.system('v4l2-ctl -d /dev/video4 --set-ctrl=exposure_time_absolute=120')
          os.system('v4l2-ctl -d /dev/video4 --set-ctrl=white_balance_temperature=3700')
          os.system('v4l2-ctl -d /dev/video4 --set-ctrl=gain=30')
          os.system('v4l2-ctl -d /dev/video4 --set-ctrl=focus_absolute=0')
      
      
      ret, img = self.cap.read()
      # cv2.imshow('img',img)
      
      if ret == True:
        
        output, location, ObFlag = self.pose_estimation(img,ARUCO_DICT[aruco_type],self.Cam_Mtrx, self.Distort,self.markerSize,self.detectObstacles)
        if self.detectObstacles!=0:
          FilteredContourBoxes= self.obstacle_detector(img,ObFlag)
        
        cv2.namedWindow(self.windowname,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.windowname,1728,972)
        cv2.imshow(self.windowname, output)

      key = cv2.waitKey(1) & 0xFF # pressing q will quit popup window and close capture
      if key == ord('q'):
        break

    self.cap.release()
    cv2.destroyAllWindows()
    

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100
}


def main(args=None):
  rclpy.init(args=args)
  aruco_reader = ArucoReader()
  rclpy.spin(aruco_reader)
  aruco_reader.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()

