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
import numpy as np
import cv2
import imutils
import copy
import random
from PIL import Image, ImageDraw,ImageFont
import PyKDL
from tf_conversions import posemath
class det_ob:
	def __init__(self):
		
        	self.limb = Limb()
        	self.traj_options = TrajectoryOptions()
        	self.head_display = HeadDisplay()
        	self.linear_speed = 0.1
        	self.linear_accel = 0.1
        	self.rotational_speed = 1.57 / 4
        	self.rotational_accel = 1.57 / 4
        	self.tip_name = "right_hand"
        	self.timeout = None
		self.head_display=HeadDisplay()
		self.thinking = rospy.Publisher('/thinking_gif', String, queue_size =10)		
		self.home_position = [0.65, 0.2,0.2,1.0,0.0,0.0,0.0]
		self.carts_move(self.home_position)
		self.img_counter=0		
		self.get_pose_camera()
		
				
		
		
		
	def carts_move(self,pos):
		self.traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        	traj = MotionTrajectory(trajectory_options=self.traj_options, limb=self.limb)

        	wpt_opts = MotionWaypointOptions(max_linear_speed=self.linear_speed,
                                         max_linear_accel=self.linear_accel,
                                         max_rotational_speed=self.rotational_speed,
                                         max_rotational_accel=self.rotational_accel,
                                         max_joint_speed_ratio=1.0)
        	waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self.limb)
		joint_angles = self.limb.joint_ordered_angles()
        	joint_names = self.limb.joint_names()
		endpoint_state = self.limb.tip_state(self.tip_name)#this line would save intial postion,speed torque and acc etc.of robot to the variabel
		#print(endpoint_state)
        	pose = endpoint_state.pose#This line would save the pose matrix in the varible
		pose.position.x = pos[0]
        	pose.position.y = pos[1]
        	pose.position.z = pos[2]
        	pose.orientation.x = pos[3]
	        pose.orientation.y = pos[4]
        	pose.orientation.z = pos[5]
        	pose.orientation.w = pos[6]
		pose_stamped = PoseStamped() #creates a varible in the form std_msg message
        	pose_stamped.pose = pose
		#print(pose_stamped)
		waypoint.set_cartesian_pose(pose_stamped, self.tip_name, joint_angles)
	
        	#rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())
		#print(waypoint)
        	traj.append_waypoint(waypoint.to_msg())	
		result = traj.send_trajectory(timeout=None)
		#rospy.logerr(result)
		shif_ni=[0.01,0.01,0.01,0.01,0.01,1.57]
		rot=PyKDL.Rotation.RPY(shif_ni[3],shif_ni[4],shif_ni[5])
		trans=PyKDL.Vector(shif_ni[0],shif_ni[1],shif_ni[2])
		f2=PyKDL.Frame(rot,trans)
		pose=posemath.toMsg(posemath.fromMsg(pose)*f2)
		pose_stamped = PoseStamped() #creates a varible in the form std_msg message
        	pose_stamped.pose = pose
		waypoint.set_cartesian_pose(pose_stamped, self.tip_name, joint_angles)
		traj.append_waypoint(waypoint.to_msg())
		shif_ni=[0.01,0.01,0.01,0.52,0.01,0.01]
		rot=PyKDL.Rotation.RPY(shif_ni[3],shif_ni[4],shif_ni[5])
		trans=PyKDL.Vector(shif_ni[0],shif_ni[1],shif_ni[2])
		f2=PyKDL.Frame(rot,trans)
		pose=posemath.toMsg(posemath.fromMsg(pose)*f2)
		pose_stamped = PoseStamped() #creates a varible in the form std_msg message
        	pose_stamped.pose = pose
		waypoint.set_cartesian_pose(pose_stamped, self.tip_name, joint_angles)
		traj.append_waypoint(waypoint.to_msg())	
		result = traj.send_trajectory(timeout=None)	
		if result is None:
        	    rospy.logerr('Trajectory FAILED to send')
	
        	if result.result:
        	    rospy.loginfo('Motion controller successfully finished the trajectory!')
        	else:
        	    rospy.logerr('Motion controller failed to complete the trajectory with error %s',
        	                 result.errorId)
	def red_img(self,x_loc,y_loc,obj_shap):
		cam = cv2.VideoCapture(0)
		while True:			
			_,frame=cam.read()
			frame=frame[0:480,0:422]
			hsv_org = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			lower_red=np.array([124,4,178])
    			upper_red=np.array([180,255,255])
			mask20= cv2.inRange(hsv_org, lower_red, upper_red)
			lower_red = np.array([0,86,220])
    			upper_red = np.array([23,255,255])
    			mask21=cv2.inRange(hsv_org, lower_red, upper_red)
			mask2=mask20
			#hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
			abc=cv2.absdiff(self.cam_pos_red,mask2)			
			#np.savetxt('temp.csv',abc,delimiter=",")
			print(np.count_nonzero(abc))
			#cv2.imshow('frame',frame)
			
			key=cv2.waitKey(1)
			#rospy.sleep(15)
			#if cv2.waitKey(1) == ord('q'):
       			break
		print('count',np.count_nonzero(abc))
		cam.release()		
		if(np.count_nonzero(abc)<2000):
			print('no change')
			return(0,x_loc,y_loc)
		else:
			cam = cv2.VideoCapture(0)
			self.thinking.publish("thinking")
			self.carts_move([0.65, 0.55,0.2,1.0,0.0,0.0,0.0])
			rospy.sleep(3.0)
			while True:
				_,frame=cam.read()
				cv2.imshow('frame',frame)
				key=cv2.waitKey(1)
				#rospy.sleep(15)
				#if cv2.waitKey(1) == ord('q'):
       				break
			frame=frame[0:480,70:519]
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			lower_red=np.array([124,4,178])
    			upper_red=np.array([180,255,255])
			mask20= cv2.inRange(hsv, lower_red, upper_red)
			lower_red = np.array([0,86,220])
    			upper_red = np.array([23,255,255])
    			mask21=cv2.inRange(hsv, lower_red, upper_red)
			mask2=mask20
			
			#abc = cv2.GaussianBlur(abc, (3, 3), 0)
			#abc=(abc*255).astype("uint8")
			#_, thrash_12=cv2.threshold(abc,220,255,cv2.THRESH_BINARY)
			
			#raw_input("Press Enter to continue...")
			_,contours,_=cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			for cnt in contours:				
				approx=cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
				area=cv2.contourArea(cnt)
				#x=approx.ravel()[0]
				#y=approx.ravel()[1]
				if (area>400):
					M=cv2.moments(cnt)
					x = int(M ["m10"]/M["m00"])
					y = int(M["m01"]/M["m00"])
					cv2.drawContours(frame,[approx],-1,(0,0,255),3)	
					print(len(approx))
					if obj_shap=='red_spoon' and area>1000 and area <4000:
						correct=1
						cam.release()
						cv2.destroyAllWindows()
									
						#self.cam_pos_red=mask2
						break
					elif obj_shap=='red_bowl' and area>9000 and area<15000:						
						cam.release()
						cv2.destroyAllWindows()			
						#self.cam_pos_red=mask2
						correct=1
						break
					elif obj_shap=='red_glass' and area>4000 and area<9000:						
						cam.release()
						cv2.destroyAllWindows()			
						#self.cam_pos_red=mask2
						correct=1
						break
					else:
                                                cam.release()
						cv2.destroyAllWindows()			
						#self.cam_pos_red=mask2
						correct=0
						pass
				else:
					correct=0
					x=0
					y=0
			
			cv2.destroyAllWindows()	
			rospy.sleep(2)					
			self.carts_move(self.home_position)
			rospy.sleep(2)
			cam.release()
			cam = cv2.VideoCapture(0)	
			_,frame=cam.read()
			frame=frame[0:480,0:422]
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			lower_red=np.array([124,4,178])
    			upper_red=np.array([180,255,255])
			mask20= cv2.inRange(hsv, lower_red, upper_red)
			lower_red = np.array([0,86,220])
    			upper_red = np.array([23,255,255])
    			mask21=cv2.inRange(hsv, lower_red, upper_red)
			mask2=mask20
			if correct==1:
				self.cam_pos_red=mask2
																								
			return (correct,x,y)
	def green_img(self,x_loc,y_loc,obj_shap):
		cam = cv2.VideoCapture(0)
		while True:			
			_,frame=cam.read()
			frame=frame[0:480,0:422]
			hsv_org = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			lower_green=np.array([30,75,75])
                        upper_green=np.array([80,255,255])
			mask2= cv2.inRange(hsv_org, lower_green, upper_green)
			#hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
			abc=cv2.absdiff(self.cam_pos_green,mask2)			
			#np.savetxt('temp.csv',abc,delimiter=",")
			print(np.count_nonzero(abc))
			#cv2.imshow('frame',frame)
			
			key=cv2.waitKey(1)
			#rospy.sleep(15)
			#if cv2.waitKey(1) == ord('q'):
       			break
		print('count',np.count_nonzero(abc))
		cam.release()		
		if(np.count_nonzero(abc)<2000):
			print('no change')
			return(0,x_loc,y_loc)
		else:
			cam = cv2.VideoCapture(0)
			self.thinking.publish("thinking")			
			self.carts_move([0.65, -0.2,0.2,1.0,0.0,0.0,0.0])
			rospy.sleep(3.0)
			while True:
				_,frame=cam.read()
				cv2.imshow('frame',frame)
				key=cv2.waitKey(1)
				#rospy.sleep(15)
				#if cv2.waitKey(1) == ord('q'):
       				break
			frame=frame[0:480,52:309]
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			lower_green=np.array([30,75,75])
                        upper_green=np.array([80,255,255])
			mask2= cv2.inRange(hsv, lower_green, upper_green)
			
			#abc = cv2.GaussianBlur(abc, (3, 3), 0)
			#abc=(abc*255).astype("uint8")
			#_, thrash_12=cv2.threshold(abc,220,255,cv2.THRESH_BINARY)
			
			#raw_input("Press Enter to continue...")
			_,contours,_=cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			for cnt in contours:				
				approx=cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
				area=cv2.contourArea(cnt)
				#x=approx.ravel()[0]
				#y=approx.ravel()[1]
				if (area>400):
					M=cv2.moments(cnt)
					x = int(M ["m10"]/M["m00"])
					y = int(M["m01"]/M["m00"])
					cv2.drawContours(frame,[approx],-1,(0,0,255),3)	
					print(len(approx))
					if obj_shap=='green_plate' and area>10000 and area <30000:
						correct=1
						cam.release()
						cv2.destroyAllWindows()
									
						#self.cam_pos_red=mask2
						break
					elif obj_shap=='green_knife' and area>1500 and area<4000:						
						cam.release()
						cv2.destroyAllWindows()			
						#self.cam_pos_red=mask2
						correct=1
						break
					
					else:
                                                cam.release()
						cv2.destroyAllWindows()			
						#self.cam_pos_red=mask2
						correct=0
						pass
				else:
					correct=0
					x=0
					y=0
			cv2.destroyAllWindows()	
			rospy.sleep(2)					
			self.carts_move(self.home_position)
			rospy.sleep(2)
			cam.release()
			cam = cv2.VideoCapture(0)	
			_,frame=cam.read()
			frame=frame[0:480,0:422]
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			lower_green=np.array([30,75,75])
                        upper_green=np.array([80,255,255])
			mask2= cv2.inRange(hsv,lower_green,upper_green)
			if correct==1:
				self.cam_pos_green=mask2
																	
			return (correct,x,y)		
		
	def cap_img(self):
		cam = cv2.VideoCapture(0) # camera reading
		cam.set(3,640)
		cam.set(4,480)
		col_ratio=.295/640
		row_ratio=0.266/480
		col_loc=[]
		row_loc=[]
		shape_obj=[]
		col_loc_r=0
		row_loc_r=0
		col_loc_c=0
		col_loc_c=0	
		while True:
			_,frame=cam.read() # take a single pic out of the cam
			frame=frame[0:480,0:422] #camera is 640X480 - reducing due to small area
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # seperate Hue Saturation Value - easier for seperation
			lower_blue = np.array([90, 50, 50])
                        upper_blue = np.array([130, 255, 255])
                        lower_green=np.array([30,75,75])
                        upper_green=np.array([80,255,255])
                        lower_red=np.array([124,4,178])
    			upper_red=np.array([180,255,255])
			mask20= cv2.inRange(hsv, lower_red, upper_red) # making mask for red color 
			lower_red = np.array([0,86,220])
    			upper_red = np.array([23,255,255])
    			mask21=cv2.inRange(hsv, lower_red, upper_red)
			mask2=mask20
                        mask = cv2.inRange(hsv, lower_blue, upper_blue) # making mask for blue
                        mask1= cv2.inRange(hsv, lower_green, upper_green)
                        _,contours,_=cv2.findContours(mask1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #finding perimeter (heikef) green
			print(len(contours))			
			for cnt in contours:				#for each object
				approx=cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True) #drawing a particular shape
				area=cv2.contourArea(cnt) #calculate area
				if (area>400): #remove noise
					M=cv2.moments(cnt) # calculate middle points
					x = int(M ["m10"]/M["m00"]) #x,y middle calculate
					y = int(M["m01"]/M["m00"])
					cv2.drawContours(frame,[approx],-1,(0,0,255),3) # drawing on frame	
					print(x)
					if (area>15000 and area <30000): #plate
						shape_obj_t='green_plate'
						col_loc_t=x
						row_loc_t=y
						col_loc.append(col_loc_t)
						row_loc.append(row_loc_t)
						shape_obj.append('green_plate')
					elif (area>1500 and area<4000): #knife
						shape_obj_r='green_knife'
						col_loc_r=x
						row_loc_r=y
						col_loc.append(col_loc_r)
						row_loc.append(row_loc_r)
						shape_obj.append('green_knife')
			_,contours,_=cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #red
			print(len(contours))			
			for cnt in contours:				
				approx=cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
				area=cv2.contourArea(cnt)
				#x=approx.ravel()[0]
				#y=approx.ravel()[1]
				if (area>400):
					M=cv2.moments(cnt)
					x = int(M ["m10"]/M["m00"])
					y = int(M["m01"]/M["m00"])
					cv2.drawContours(frame,[approx],-1,(0,0,255),3)	
					print(x)
					if (area>1000 and area <4000 and 'red_spoon' not in shape_obj):
						shape_obj_t='red_spoon'
						col_loc_t=x
						row_loc_t=y
						col_loc.append(col_loc_t)
						row_loc.append(row_loc_t)
						shape_obj.append('red_spoon')
					elif (area>9000 and area<15000):
						shape_obj_r='red_bowl'
						col_loc_r=x
						row_loc_r=y
						col_loc.append(col_loc_r)
						row_loc.append(row_loc_r)
						shape_obj.append('red_bowl')
					elif (area>4000 and area<9000):
						shape_obj_c='red_glass'
						col_loc_c=x
						row_loc_c=y
						col_loc.append(col_loc_c)
						row_loc.append(row_loc_c)
						shape_obj.append('red_glass')
			cv2.imshow('foreground',frame)
			cv2.imshow('thras',mask)
			rospy.sleep(5)
			key=cv2.waitKey(1)
			rospy.sleep(1)					
			break
		raw_input("Press Enter to continue...") 
		self.org_frame=hsv #initial scanning ("change / no change")
		self.cam_pos_blue=mask
		self.cam_pos_green=mask1
		self.cam_pos_red=mask2
		cam.release()
		cv2.destroyAllWindows()
		print(shape_obj)
		print(col_loc)
		print(row_loc)
		return(col_loc,row_loc,shape_obj)
		
		
	def get_pose_camera(self):
		self.prob=[0.2,0.2,0.2,0.2,0.2]
		self.square=[]
		self.triangle=[]
		self.circle=[]
		pose=copy.copy(self.home_position)

		col_loc,row_loc,shape_obj=self.cap_img()
		column_location=np.array(col_loc)
		x_loca=(0.06409*column_location+0.00903)/100
		row_location=np.array(row_loc)
		y_loca=(-0.0001*(row_location**2)+0.1321*row_location+0.4269)/100 
		x_loc=x_loca.flatten()
		y_loc=y_loca.flatten()
		self.col_l=col_loc
		self.row_l=row_loc
		self.obj_l=shape_obj
		print(x_loc,y_loc)
		
        	for i in range (0,len(shape_obj)):	
			#pose[0]=0.62+x_loc[i]
			#print(pose[0])
			#pose[1]=self.home_position[1]+(0.47-y_loc[i])
			#self.carts_move(pose)
			self.generate_voice(i)			
			rospy.sleep(0.5)
			#print(pose)
			
			#$a=self.det_obj_img()
			count=0
			#raw_input("Press Enter to continue...")
			#if (a==1):
			#	print ("The object is found")
			#	break
			#rospy.sleep(0.5)
		#print("The object is not found")
		
		print(self.square)
		print(self.triangle)
		print(self.circle)
		print(self.col_l)
		print(self.row_l)
		print('weights',self.prob)
	
	def arange_boxes(self,i,change,level,correct):		
		inside=0
		if(self.obj_l[i]=='red_glass' ):
			inside=0
			self.generate_level_texat(level,i,'glass','C','placed in the middle')
			change,x_loc,y_loc=self.red_img(self.col_l[i],self.row_l[i],self.obj_l[i])
			
			
			if (change==1):
                                correct=1
                                inside =0
			else:
				inside=0
				correct=0
		elif(self.obj_l[i]=='red_bowl'  ):
			inside=0
			self.generate_level_texat(level,i,'bowl','D','placed in the middle')
			change,x_loc,y_loc=self.red_img(self.col_l[i],self.row_l[i],self.obj_l[i])
			
			
			if (change==1):
                                correct=1
                                inside =0
			else:
				inside=0
				correct=0
		elif(self.obj_l[i]=='red_spoon' ):
			inside=0
			self.generate_level_texat(level,i,'spoon','E','placed in the middle')
			change,x_loc,y_loc=self.red_img(self.col_l[i],self.row_l[i],self.obj_l[i])
			
			
			if (change==1):
                                correct=1
                                inside =0
			else:
				inside=0
				correct=0
		elif(self.obj_l[i]=='green_knife' ):
			inside=0
			self.generate_level_texat(level,i,'knife','B','placed in the middle')
			change,x_loc,y_loc=self.green_img(self.col_l[i],self.row_l[i],self.obj_l[i])	
			
			if (change==1):
                                correct=1
                                inside =0
			else:
				inside=0
				correct=0
		elif(self.obj_l[i]=='green_plate' ):
			inside=0
			self.generate_level_texat(level,i,'plate','A','placed in the middle')
			change,x_loc,y_loc=self.green_img(self.col_l[i],self.row_l[i],self.obj_l[i])	
			
			if (change==1):
                                correct=1
                                inside =0
			else:
				inside=0
				correct=0
				
		
		return change,correct,inside
								
	def generate_level_texat(self,level,i,initial,final,row):
		if(level==0):
			text=intial
		elif(level==1):
			
			text='move '+initial
		elif(level ==2):
			text='move '+initial+'\n'+row
		elif(level == 3):
			text='move '+initial +' to '+'\n'+ final
		else:
			text='move '+initial+'\n'+row+'\n'+' to '+final
		img = Image.new('RGB', (1024, 600), color = (73, 109, 137))
		font = ImageFont.truetype(font="/usr/share/fonts/truetype/freefont/FreeMono.ttf", size=84)
	 	d = ImageDraw.Draw(img)
		d.text((50,50), text, font=font,fill=(255,255,0))			
		img.save('pil_text.jpg')	
		self.location="pil_text.jpg"
		loc=self.location
		#print(loc)
		self.thinking.publish("instruction")
		#self.head_display.display_image(loc,False,5)
		rospy.sleep(8)
	def learning_algo(self, level,i,change,correct):
		penalty=-3
		reward=3
		learning_rate=0.1
		
		#rospy.sleep(15)
		##decide the action taken by user answer true or false
		change,correct,inside = self.arange_boxes(i,change,level,correct)
		if (correct==1):
			text="Good Job"
			r=reward
			self.thinking.publish("well_done")
		else:
			text="Wrong Action \n Place the object\n to orignal position"
			r=penalty
		
			img = Image.new('RGB', (1024, 600), color = (73, 109, 137))
			font = ImageFont.truetype(font="/usr/share/fonts/truetype/freefont/FreeMono.ttf", size=84)
	 		d = ImageDraw.Draw(img)
			d.text((50,50), text, font=font,fill=(255,255,0))			
			img.save('pil_text.jpg')	
			self.location="pil_text.jpg"
			loc=self.location
			self.thinking.publish("instruction")
		#print(loc)
		#self.head_display.display_image(loc,False,5)
		rospy.sleep(8)
				#voice message redo action		
		if(self.prob[level]+r>0):
			self.prob[level]=(self.prob[level]+r*learning_rate*self.prob[level])
		for j in range(len(self.prob)):
			if j !=level:
				self.prob[j]=self.prob[j]+self.prob[j]*(1-r)*learning_rate
		sum_whole=sum(self.prob)
		for j in range(0, len(self.prob)):
			self.prob[j]=self.prob[j]/sum_whole
			#print(self.prob)
		with open("output.csv", "a") as f:   # use 'a' instead of 'ab'
    			np.savetxt(f, self.prob,  fmt="%f", delimiter=",")
    			f.write("\n")
		return change,correct,inside
	def generate_voice(self,i):
		pen=[]
		correct=0
		change=0
		a=0
		while True:
			level=self.choose_level()
			#generate voice for the level
			#rospy.sleep(15)
			#print(level)
			change,correct,inside=self.learning_algo(level,i,change,correct)
			if(len(pen)==5 or correct==1 or inside or a==10 or correct==1 ):
				break
			else:
				a=a+1
				if (level not in pen):
					pen.append(level)
				else:
					pass
			
			

	def choose_level(self):
		p = random.random()
		data_lev=copy.copy(self.prob)
		l_sort=[{'level':0,'prob':data_lev[0]},{'level':1,'prob':data_lev[1]},{'level':2,'prob':data_lev[2]},{'level':3,'prob':data_lev[3]},{'level':4,'prob':data_lev[4]}]
		l_sort.sort(key=lambda x:x['prob'])
		if(0<p<l_sort[0]['prob']):
			return l_sort[0]['level']
		elif(l_sort[0]['prob']<p<(l_sort[0]['prob']+l_sort[1]['prob'])):
			return l_sort[1]['level']
		elif((l_sort[0]['prob']+l_sort[1]['prob'])<p<(l_sort[0]['prob']+l_sort[1]['prob']+l_sort[2]['prob'])):
			return l_sort[2]['level']
		elif((l_sort[0]['prob']+l_sort[1]['prob']+l_sort[2]['prob'])<p<(l_sort[0]['prob']+l_sort[1]['prob']+l_sort[2]['prob']+l_sort[3]['prob'])):
			return l_sort[3]['level']
		elif((l_sort[0]['prob']+l_sort[1]['prob']+l_sort[2]['prob']+l_sort[3]['prob'])<p<(l_sort[0]['prob']+l_sort[1]['prob']+l_sort[2]['prob']+l_sort[3]['prob']+l_sort[4]['prob'])):
			return l_sort[4]['level']
		

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('Move_Cart', anonymous=False)

    try:
         Obj_clas= det_ob()
    except rospy.ROSInterruptException:
        pass
	
		
