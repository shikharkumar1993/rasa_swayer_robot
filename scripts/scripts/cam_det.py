#! /usr/bin/env python
import numpy as np
import cv2
#from __future__ import print_function
cap = cv2.VideoCapture(0)
from PIL import Image
while True:
    ret, frame = cap.read()
    width = int(cap.get(3))
    height = int(cap.get(4))
    #print (frame)
    #frame=frame[0:480,0:422]
    #frame=frame[0:480,70:519]
    #frame=frame[67:480,120:570]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])
    lower_green=np.array([30,75,75])
    upper_green=np.array([80,255,255])
    lower_red=np.array([124,4,178])
    upper_red=np.array([180,255,255])
    #mask = cv2.inRange(hsv, lower_red, upper_red)
    #lower_red = np.array([0,86,220])
    #upper_red = np.array([23,255,255])
    #mask1=cv2.inRange(hsv, lower_red, upper_red)
    mask=cv2.inRange(hsv,lower_green,upper_green)
    #mask=mask1+mask0
    contours,_=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #print(len(contours))
    for cnt in contours:
    	approx=cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    	area=cv2.contourArea(cnt)
    	if (area>400):
    		M=cv2.moments(cnt)
    		x = int(M ["m10"]/M["m00"])
    		y = int(M["m01"]/M["m00"])
    		#x1,y1,w,h=cv2.boundingRect(cnt)
    		cv2.drawContours(frame,[approx],-1,(0,0,255),3)
    		cv2.putText(frame,str(area),(x,y),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,0))
    result = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('orignal frame', frame)
    cv2.imshow('frame', result)
    cv2.imshow('mask', mask)
    if cv2.waitKey(1) == ord('q'):
    	break
cap.release()
cv2.destroyAllWindows()
