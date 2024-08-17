#! /usr/bin/env python
"""A program to move the robot from pos A to pos B"""

#! /usr/bin/env python
"""A program to move the robot from pos A to pos B"""

import rospy
import multiprocessing
#from libezgripper import create_connection, Gripper
import socket
import time
from std_msgs.msg import Int32, String
from intera_interface import Limb, HeadDisplay, Head
from intera_motion_msgs.msg import TrajectoryOptions



from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import cv2
import tf
from pyzbar import pyzbar 
class barcode_read_qr_code():

    
     
    def __init__(self,mid_x,mid_y):
        self.mid_x= mid_x
        self.mid_y= mid_y
        self.data=0
        self.number_times_error=0

    def error_cubes_out_range(self):
        self.video_reader()
        index=0
        data=""
        max_x=0
        min_x=10000
        max_y=0
        min_y=10000
        try:
            print("abcd")        
            for i in range(0,len(self.barcodes)):
                data=self.barcodes[i].data.decode("utf-8")
                if(data=="shelf1_2" ):
                    index=i
            print(index)     
            self.data=data
            for i in range(0,len(self.barcodes[index].polygon)):                
                if(self.barcodes[index].polygon[i].x>max_x):
                    max_x=self.barcodes[index].polygon[i].x
                if(self.barcodes[index].polygon[i].x<min_x):
                    min_x=self.barcodes[index].polygon[i].x
                if(self.barcodes[index].polygon[i].y>max_y):
                    max_y=self.barcodes[index].polygon[i].y
                if(self.barcodes[index].polygon[i].y<min_y):
                    min_y=self.barcodes[index].polygon[i].y
            self.mid_x=min_x+(max_x-min_x)/2
            self.mid_y=min_y+(max_y-min_y)/2              
            print("@@@@@@",self.mid_x,self.mid_y)

        except:
            print("error no cubes on table")    
    def error_cubes(self):
        self.video_reader()
        index=0
        data=""
        max_x=0
        min_x=10000
        max_y=0
        min_y=10000
        try:
            print("abcd")        
            for i in range(0,len(self.barcodes)):
                data=self.barcodes[i].data.decode("utf-8")
                if(data=="shelf2_1" or data=="shelf2_3"):
                    index=i
            print(index)     
            self.data=data
            for i in range(0,len(self.barcodes[index].polygon)):                
                if(self.barcodes[index].polygon[i].x>max_x):
                    max_x=self.barcodes[index].polygon[i].x
                if(self.barcodes[index].polygon[i].x<min_x):
                    min_x=self.barcodes[index].polygon[i].x
                if(self.barcodes[index].polygon[i].y>max_y):
                    max_y=self.barcodes[index].polygon[i].y
                if(self.barcodes[index].polygon[i].y<min_y):
                    min_y=self.barcodes[index].polygon[i].y
            self.mid_x=min_x+(max_x-min_x)/2
            self.mid_y=min_y+(max_y-min_y)/2              
            print("@@@@@@",self.mid_x,self.mid_y)

        except:
            print("error no cubes on table")
    def detect_number_times_error_range(self,msg_from_main):
        self.number_times_error=msg_from_main                
        self.video_reader()
        
    def video_reader(self):
        self.cam = cv2.VideoCapture(0)
        self.detector = cv2.QRCodeDetector()
        self.calib_path="/home/sawyer/camera_calibration/calibrationfiles/"
        self.mtx   = np.loadtxt(self.calib_path+'cameraMatrix.txt', delimiter=',')
        self.dist   = np.loadtxt(self.calib_path+'cameraDistortion.txt', delimiter=',')
        self.qcd = cv2.QRCodeDetector()
        max_x=0
        min_x=10000
        max_y=0
        min_y=10000
        ind=0
        count=0
        dt=""
        while True:
            _, img = self.cam.read()
            h,  w = img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
            img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
            # crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]
            data=0
            #img=cv2.flip(img, 1)
            try:
                self.barcodes=pyzbar.decode(img)
                if (self.number_times_error!=0 and self.barcodes[0].data.decode("utf-8")=="shelf1_2"):
                    ind=1
                else:
                    ind=0              
                 
                       
                            
                    
                
                print(self.barcodes)
                data=self.barcodes[ind].data.decode("utf-8")
                
                for i in range(0,len(self.barcodes[0].polygon)):
                    if(self.barcodes[ind].polygon[i].x>max_x):
                        max_x=self.barcodes[ind].polygon[i].x
                    if(self.barcodes[ind].polygon[i].x<min_x):
                        min_x=self.barcodes[ind].polygon[i].x
                    if(self.barcodes[ind].polygon[i].y>max_y):
                        max_y=self.barcodes[ind].polygon[i].y
                    if(self.barcodes[ind].polygon[i].y<min_y):
                        min_y=self.barcodes[ind].polygon[i].y
                    
                self.mid_x=min_x+(max_x-min_x)/2
                self.mid_y=min_y+(max_y-min_y)/2
                print("mid_x",self.mid_x)
                print("mid_y",self.mid_y)
            except:
                    print("Object not found")
            try:
                if data:
                    print("QR Code detected-->", data)     
            
                    break
                
                    
            except:
                print("data not recorded")
            cv2.imshow("img", img)
            cv2.imshow("new_frame",dst)    
            if cv2.waitKey(1) == ord("Q"):
                break
        self.data=data    
        print(data)
        print("x",self.mid_x)
        print("y",self.mid_y)
        self.cam.release()
        cv2.destroyAllWindows()
def main():
    abc=barcode_read_qr_code(0,0)
    abc.detect_number_times_error_range(1)
if __name__=="__main__":
    main()