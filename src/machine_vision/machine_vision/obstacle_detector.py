import cv2
import numpy as np
def nothing(x):
    # any operation
    pass
cap = cv2.VideoCapture(-1)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
#cap.set(cv)
# cap.set(cv2.CAP_PROP_CONTRAST,1)
cap.set(cv2.CAP_PROP_EXPOSURE,-1)
cap.set(cv2.CAP_PROP_BRIGHTNESS,100)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print(width, height, fps)

while cap.isOpened():

    ret, img = cap.read()
    grey=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    retval,thresh=cv2.threshold(grey,130,255,cv2.THRESH_BINARY)
 
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0,255,0), 2)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()