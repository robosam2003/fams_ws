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
import csv
from time import sleep
import subprocess



class ArucoReader(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_cdreader')
    self.get_logger().info('Aruco Reader Node Started')

    # Publisher Setup
    self.moblocation_pub = self.create_publisher(Point,'nexus1/aruco_tf',10)
    self.partlocation_pub=self.create_publisher(Vision,'VisionLocations',10)
    self.video_publisher=self.create_publisher(Image,'video_stream',10)

    # Create a VideoCapture object and set parameters
    camType=1
    self.camera_setup(camType)

    aruco_type = "DICT_4X4_100" #Is looking for 4x4 only

    self.main_loop(aruco_type,camType)

  def camera_setup(self,CamIndex):
    
    # self.get_logger().info('Querying Camera Device Paths...')
    # command="udevadm info --query=property --name=/dev/video2 | grep ID_SERIAL_SHORT"
    # os.system(command)
    # result=os.popen(command).read()
    # result=result.strip()
    # if result=="ID_SERIAL_SHORT=9A6239BF":
    #   maincam=2
    #   workcam1=0
    # elif result=="ID_SERIAL_SHORT=38A907E0":
    #   maincam=0
    #   workcam1=2
    
    # print(maincam)

    match CamIndex:
      case 0:
        self.get_logger().info('Setting Up Main Camera...')
        
        self.cap = cv2.VideoCapture(-1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS,30)
        
        fps=self.cap.get(cv2.CAP_PROP_FPS)
        print(fps)
        width=self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        print(width)
        
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=0')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=130')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=3700')
        os.system('v4l2-ctl -d /dev/video0 --set-ctrl=gain=30')
        
      case 1:
        self.get_logger().info('Setting Up Workstation 1 Camera...')
        
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        self.cap.set(cv2.CAP_PROP_FPS,30)
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_automatic=0')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=exposure_time_absolute=150')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_temperature=3700')
        os.system('v4l2-ctl -d /dev/video2 --set-ctrl=gain=50')


        fps=self.cap.get(cv2.CAP_PROP_FPS)
        print(fps)
        width=self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        print(width)
      case 2:
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=auto_exposure=1')
        os.system('v4l2-ctl -d /dev/video4 --set-ctrl=white_balance_automatic=0')
    return self.cap
        

  def obstacle_initial(self):
    Ob_ids=[8,9,10,11] # IDs associated to fixed obstacles (workstation, loading area)
    sizeWorkstation=[0.5,0.5]
    sizeLoadingArea=[0.6,0.4]
    return Ob_ids, sizeWorkstation, sizeLoadingArea
  
  
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


  def pose_estimation(self,frame,aruco_dict_type,camera_matrix, distortion_vector,markerSize,origin,switch):
    anti_ob_flag=[]
    locations=[]
    id_msg=[]
    loc_msg=[]
    mobilelocation_msg=Point()
    
    part_id_msg=[]
    part_loc_msg=[]
    partlocation_msg=Vision()

    # ob_id_msg=[]
    # ob_loc_msg=[]
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,cameraMatrix=camera_matrix,distCoeff=distortion_vector)
   
    if len(corners) > 0:
      for i in range(0, len(ids)):
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerSize, camera_matrix, distortion_vector)         
        cv2.aruco.drawDetectedMarkers(frame, corners,ids) 
        cv2.aruco.drawAxis(frame, camera_matrix, distortion_vector, rvec, tvec, 0.03)  

        locations=tvec-origin
        x=(round(locations[0,0,0],5))
        y=(round(locations[0,0,1],5))
        z=(round(locations[0,0,2],5))

        rmat=cv2.Rodrigues(rvec)[0]
        angles=self.rotationMatrixToEulerAngles(rmat)
        yaw=round((angles[2]),4)
        
        if switch==1:
          if ids[i,0] in (0,1):
            id_msg.append(int(ids[i,0]))
            loc_msg.extend([x,y,yaw])
            anti_ob_flag.append(corners[i]) # change for 2 robots to work
                
            mobilelocation_msg.x=x
            mobilelocation_msg.y=y
            mobilelocation_msg.z=yaw
          
        
        if switch==0:
          if ids[i,0]==81:
            print(tvec)
            origin_x=tvec[0][0][0]
            origin_y=tvec[0][0][1]
            origin_z=tvec[0][0][2]
            print(origin_x,origin_y,origin_z)
            
            originmarkerpoint=np.array(((x),(y),(origin_z),1))
            
            Transf_to_Arm=np.array(((math.cos(yaw), -math.sin(yaw), 0, 0.185),
                               (math.sin(yaw), math.cos(yaw), 0, 0),
                               (0, 0, 1, 0),
                               (0, 0, 0, 1)))
            new_origin=np.dot(Transf_to_Arm,originmarkerpoint)
            
            
          if ids[i,0] in range(82,91):
            part_id_msg.append(int(ids[i,0]))
            part_loc_msg.extend([x,y,yaw])
          partlocation_msg.part_id=part_id_msg
          partlocation_msg.part_location=part_loc_msg

      # self.get_logger().info('{}:{}'.format("Publishing Location for Part IDs",partlocation_msg))
      self.partlocation_pub.publish(partlocation_msg)
      self.moblocation_pub.publish(mobilelocation_msg)
      # self.get_logger().info('{}:{}'.format("Publishing",mobilelocation_msg))   

          # if ids[i,0] in (8,9):
          #   ob_id_msg.append(int(ids[i,0]))
          #   ob_loc_msg.extend([x,y,yaw])
          #   a,SizeW,SizeL = self.obstacle_initial()
          #   oppositeCorner=x+SizeW[0],y+SizeW[1]
      # location_msg.obstacle_id=ob_id_msg
      # location_msg.obstacle_location=ob_loc_msg
      
      

      

      # msg="x: "+ str(x) + "m" +"  y: " +str(y)+"m"+" yaw: " +str(yaw)+"rads" # Code to show location of marker- Keep commented unless for debugging purposes
      # cv2.putText(frame, msg,(210,100),cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0))
      # cv2.putText(frame, '-y  0 rads',(970,50),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))
      # cv2.putText(frame, '+y  +-pi rads',(970,1000),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))
      # cv2.putText(frame, '-x -pi/2 rads',(20,520),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))
      # cv2.putText(frame, '-x +pi/2 rads',(1800,520),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0))
      # cv2.line(frame, (0,540),(1920,540),(255,0,0),1)
      # cv2.line(frame, (960,0),(960,1080),(255,0,0),1)
    else:
      locations=0
    return frame, locations, anti_ob_flag

  def obstacle_detector(self,frame,anti_ob_flag):
    FilteredContours=[]
    FilteredBoxes=[]
    intrinsic_camera = np.array(((5.14225115e+03, 0.00000000e+00, 1.30071631e+03),(0, 5.21253566e+03,7.22183264e+02),(0,0,1)))
    distortion = np.array((2.51186484e-01, -5.65362473e+00,  1.50035516e-02,  1.11397010e-02,1.36994424e+01))

    # h,  w = frame.shape[:2]
    # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(intrinsic_camera, distortion, (w,h), 1, (w,h))
    # # undistort
    # dst = cv2.undistort(frame, intrinsic_camera, distortion, None, newcameramtx)
    grey=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    retval,thresh=cv2.threshold(grey,180,255,cv2.THRESH_BINARY)
    # cv2.namedWindow("thresh", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('thresh', 1700, 1080)   
    # cv2.imshow('thresh',thresh)
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
      area=cv2.contourArea(cnt)
      if area > 660:
        M=cv2.moments(cnt)
        scale=1/535
        cx = ((M['m10']/M['m00'])-960)*scale
        cy = ((M['m01']/M['m00'])-540)*scale
        # print(cx,cy)
        if int((M['m10']/M['m00']))>20:
          rect = cv2.minAreaRect(cnt)
          box = cv2.boxPoints(rect)
          box = np.int0(box)
          #print(box)
          
          match len(anti_ob_flag):
            case 1:
              Mark_x1=anti_ob_flag[0][0][0][0]
              Mark_y1=anti_ob_flag[0][0][0][1]
              check=cv2.pointPolygonTest(box,[Mark_x1,Mark_y1],measureDist=False)
              if check==-1:
                FilteredContours.append(cnt)
                FilteredBoxes.append(box)
                cv2.drawContours(frame,[box],0,(0,0,255),2)
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
                cv2.drawContours(frame,[box],0,(0,0,255),2)
            case 0:
              FilteredContours.append(cnt)
              FilteredBoxes.append(box)
              cv2.drawContours(frame,[box],0,(0,0,255),2)
            case _:
              print("Anti-Obstacle Flag length error")
    cv2.drawContours(frame, contours=FilteredContours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    return FilteredBoxes
    

  def main_loop(self,aruco_type,camera_type):
    print(camera_type)
    if camera_type==0:
      Cam_Mtrx = np.array(((5.14225115e+03, 0.00000000e+00, 1.30071631e+03),(0, 5.21253566e+03,7.22183264e+02),(0,0,1)))
      Distort = np.array((2.51186484e-01, -5.65362473e+00,  1.50035516e-02,  1.11397010e-02,1.36994424e+01))
      markerSize=0.15
      origin=[-0.64196,-0.341637,9.68856]
      detectObstacles=1
    elif camera_type==1:
      Cam_Mtrx = np.array(((1.21665111e+03, 0, 6.54768787e+02),(0, 1.21478888e+03, 5.00652432e+02),(0,0,1)))
      Distort = np.array((0.04315798,  0.50036972, -0.01800276, -0.00592732, -1.26928846))
      markerSize=0.0325
      origin=[(-0.03480-0.185),  0.28, -0.0202]
      detectObstacles=0
    else:
      self.get_logger().info('Camera Selection Fail Abort')
      self.cap.release()
      cv2.destroyAllWindows()
    
    while self.cap.isOpened():
      # os.system('v4l2-ctl -d /dev/video{} --set-ctrl=exposure_time_absolute=130'.format(2))
      # os.system('v4l2-ctl -d /dev/video{} --set-ctrl=white_balance_temperature=3700'.format(2))
      # os.system('v4l2-ctl -d /dev/video{} --set-ctrl=gain=30'.format(2))
      
      ret, img = self.cap.read()
      # cv2.imshow('cam',img)
      
      output, location, ObFlag = self.pose_estimation(img,ARUCO_DICT[aruco_type],Cam_Mtrx, Distort,markerSize,origin,detectObstacles)
      
      # FilteredContourBoxes= self.obstacle_detector(img,ObFlag)
      cv2.imshow('Estimated Pose', output)
      
      
      
      
      
    

      if ret == True:
        # Print debugging information to the terminal
        # self.get_logger().info("Publishing Video Frame")
             
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        self.br = CvBridge()
        # vid_msg=self.br.cv2_to_imgmsg(output)
        # self.video_publisher.publish(vid_msg)
        
      
      

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