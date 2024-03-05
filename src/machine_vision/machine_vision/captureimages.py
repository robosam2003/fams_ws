import numpy as np
import cv2
import sys
import time
import os

import cv2

cap = cv2.VideoCapture(-1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))


cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print(width, height, fps)


os.system('v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1')

os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0')



cv2.namedWindow("test")

img_counter = 0

while True:
    ret, frame = cap.read()
    os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=240')
    os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=3700')
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow("test", frame)

    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "opencv_frame_{}.png".format(img_counter)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1

cap.release()

cv2.destroyAllWindows()