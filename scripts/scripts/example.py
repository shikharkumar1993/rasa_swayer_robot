#! /usr/bin/env python
"""A program to move the robot through a sequence of poses using the gripper function"""

import rospy
import time
import tf
import numpy as np
from intera_interface import Limb
from intera_motion_msgs.msg import TrajectoryOptions
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
#from shelfs_movement import qr_code_recog
from geometry_msgs.msg import PoseStamped
from gripper import gripperChange  # we have the gripperChange function implemented in the gripper module
#from shelfs_movement import
import cv2
from shelfs_movement import robot_movement
from  qr_code_reader import barcode_read_qr_code 
import requests
import json
from PIL import Image
from getkey import getkey
from PIL import ImageDraw
from PIL import ImageFont
from intera_interface import HeadDisplay
import os

def go_to_pose(pose_stamped):
    limb = Limb()
    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj = MotionTrajectory(trajectory_options=traj_options, limb=limb)
    wpt_opts = MotionWaypointOptions(max_linear_speed=0.5,
                                     max_linear_accel=0.5,
                                     max_rotational_speed=1.57,
                                     max_rotational_accel=1.57,
                                     max_joint_speed_ratio=1.0)
    
    tip_name = 'right_hand'
    joint_angles = limb.joint_ordered_angles()
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
    waypoint.set_cartesian_pose(pose_stamped, tip_name, joint_angles)
    traj.append_waypoint(waypoint.to_msg())
    result = traj.send_trajectory(timeout=None)
    if result is None or not result.result:
         rospy.logerr('Failed to move to intial postion. Error: %s', result.errorId)
         return

def main():
   rospy.init_node('perform_sequence')
   for i in range(0,6): #need 8
    
        
    # Open the gripper
    rospy.loginfo('Opening the gripper...')
    gripperChange(0)
    time.sleep(1)

    # Pose 1
    initial_pose=[0.7,0.0,0.40,np.pi*10/180,np.pi*180/180,np.pi*180/180]
    
    inital_quaternion = tf.transformations.quaternion_from_euler(initial_pose[3], initial_pose[4], initial_pose[5])
   
    # Go to initial Pose 
    rospy.loginfo('Moving to intial postion...')
    pose_stamped1 = PoseStamped()
    pose_stamped1.pose.position.x = initial_pose[0]
    pose_stamped1.pose.position.y = initial_pose[1]
    pose_stamped1.pose.position.z = initial_pose[2]
    pose_stamped1.pose.orientation.x = inital_quaternion[0]
    pose_stamped1.pose.orientation.y = inital_quaternion[1]
    pose_stamped1.pose.orientation.z = inital_quaternion[2]
    pose_stamped1.pose.orientation.w = inital_quaternion[3]
    go_to_pose(pose_stamped=pose_stamped1)
    #rospy.sleep(10)

    

    # Go to middle of qr code
    abc=barcode_read_qr_code(0,0)
    ab=robot_movement()
    abc.video_reader()
    data=abc.data
    x_cord=fixed_x_cordination(abc.mid_x)
    y_cord=fixed_y_cordination(abc.mid_y)
    print(x_cord)
    print(y_cord)
    print(data)

    middle_pose=[x_cord,y_cord,-0.02,np.pi*0/180,np.pi*180/180,np.pi*180/180]
    middle_quaternion = tf.transformations.quaternion_from_euler(middle_pose[3], middle_pose[4], middle_pose[5])

    rospy.loginfo('Moving to middle of the qr code ..')
    pose_stamped2 = PoseStamped()
    pose_stamped2.pose.position.x = middle_pose[0]
    pose_stamped2.pose.position.y = middle_pose[1]
    pose_stamped2.pose.position.z = middle_pose[2]
    pose_stamped2.pose.orientation.x = middle_quaternion[0]
    pose_stamped2.pose.orientation.y = middle_quaternion[1]
    pose_stamped2.pose.orientation.z = middle_quaternion[2]
    pose_stamped2.pose.orientation.w = middle_quaternion[3]
    go_to_pose(pose_stamped=pose_stamped2)
    
    
    #catsh the object
    rospy.loginfo('closing the gripper...')
    gripperChange(1)


    #go to upper pose
    upper_pose=[x_cord,y_cord,0.15,np.pi*0/180,np.pi*180/180,np.pi*180/180]
    upper_quaternion = tf.transformations.quaternion_from_euler(upper_pose[3], upper_pose[4], upper_pose[5])

    rospy.loginfo('Moving to upper pose  ..')
    pose_stamped3 = PoseStamped()
    pose_stamped3.pose.position.x = upper_pose[0]
    pose_stamped3.pose.position.y = upper_pose[1]
    pose_stamped3.pose.position.z = upper_pose[2]
    pose_stamped3.pose.orientation.x = upper_quaternion[0]
    pose_stamped3.pose.orientation.y = upper_quaternion[1]
    pose_stamped3.pose.orientation.z = upper_quaternion[2]
    pose_stamped3.pose.orientation.w = upper_quaternion[3]
    go_to_pose(pose_stamped=pose_stamped3) 


#########################################################################################
    ############## this is for training , because the printed and stacked qr codes on the cubes temporary now
    ################## the data in the qr we will print will be like this : 
    ##################   shelf1_1 , shelf1_2 , shelf1_3 
    ##################   shelf2_1 , shelf2_2 , shelf2_3
    ##################   shelf3_1 , shelf3_2 , shelf3_3
#########################################################################################
     
    # Go to needed shelf
    if (data=="shelf1"):
            # rotate
            pose2 = [x_cord,y_cord,0.15,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
            # going in the shelf and release
            pose3 = [0.7,-0.417, 0.02 ,np.pi*(-20/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            #going back from the shelf 
            pose4 = [0.7,-0.217, 0.02 ,np.pi*(-20/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])

    elif(data=="shelf2"):
            # rotate
            #TODO_GABRIEL: put the cube down

            down_pose=[x_cord,y_cord,-0.02,np.pi*0/180,np.pi*180/180,np.pi*180/180]
            down_quaternion = tf.transformations.quaternion_from_euler(down_pose[3], down_pose[4], upper_pose[5])

            rospy.loginfo('Moving to down pose  ..')
            pose_stamped7 = PoseStamped()
            pose_stamped7.pose.position.x = down_pose[0]
            pose_stamped7.pose.position.y = down_pose[1]
            pose_stamped7.pose.position.z = down_pose[2]
            pose_stamped7.pose.orientation.x = down_quaternion[0]
            pose_stamped7.pose.orientation.y = down_quaternion[1]
            pose_stamped7.pose.orientation.z = down_quaternion[2]
            pose_stamped7.pose.orientation.w = down_quaternion[3]
            go_to_pose(pose_stamped=pose_stamped7)

            rospy.loginfo('Opening the gripper...')
            gripperChange(0)

            down_pose=[x_cord,y_cord,-0.02,np.pi*0/180,np.pi*180/180,np.pi*180/180]
            down_quaternion = tf.transformations.quaternion_from_euler(down_pose[3], down_pose[4], upper_pose[5])

            # Go to initial Pose a
            rospy.loginfo('Moving to intial postion...')
            pose_stamped1 = PoseStamped()
            pose_stamped1.pose.position.x = initial_pose[0]
            pose_stamped1.pose.position.y = initial_pose[1]
            pose_stamped1.pose.position.z = initial_pose[2]
            pose_stamped1.pose.orientation.x = inital_quaternion[0]
            pose_stamped1.pose.orientation.y = inital_quaternion[1]
            pose_stamped1.pose.orientation.z = inital_quaternion[2]
            pose_stamped1.pose.orientation.w = inital_quaternion[3]
            go_to_pose(pose_stamped=pose_stamped1)

            # keep it in the same pose
            pose2 = [0.7,0.0,0.40,np.pi*10/180,np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
            # 
            pose3 = [0.7,0.0,0.40,np.pi*10/180,np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            # 
            pose4 = [0.7,0.0,0.40,np.pi*10/180,np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5]) 


            obser_vation,x_cordi,y_cordi=detect_error_cube(expected_shelf="shelf3",msg="hey",fail=1)
            print(obser_vation)
            print("++++++++++++++++++++++++")

            if(obser_vation==1):
                print("SUCCESS")
                pose2 = [x_cord,y_cord,0.45,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
                quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])


            elif(obser_vation==0):
                print("FAILED")

                middle_pose=[x_cordi,y_cordi,-0.02,np.pi*0/180,np.pi*180/180,np.pi*180/180]
                middle_quaternion = tf.transformations.quaternion_from_euler(middle_pose[3], middle_pose[4], middle_pose[5])

                rospy.loginfo('Moving to middle of the qr code ..')
                pose_stamped2 = PoseStamped()
                pose_stamped2.pose.position.x = middle_pose[0]
                pose_stamped2.pose.position.y = middle_pose[1]
                pose_stamped2.pose.position.z = middle_pose[2]
                pose_stamped2.pose.orientation.x = middle_quaternion[0]
                pose_stamped2.pose.orientation.y = middle_quaternion[1]
                pose_stamped2.pose.orientation.z = middle_quaternion[2]
                pose_stamped2.pose.orientation.w = middle_quaternion[3]
                go_to_pose(pose_stamped=pose_stamped2)
                
                
                #catsh the object
                rospy.loginfo('closing the gripper...')
                gripperChange(1)


                #go to upper pose
                upper_pose=[x_cord,y_cord,0.15,np.pi*0/180,np.pi*180/180,np.pi*180/180]
                upper_quaternion = tf.transformations.quaternion_from_euler(upper_pose[3], upper_pose[4], upper_pose[5])

                rospy.loginfo('Moving to upper pose  ..')
                pose_stamped3 = PoseStamped()
                pose_stamped3.pose.position.x = upper_pose[0]
                pose_stamped3.pose.position.y = upper_pose[1]
                pose_stamped3.pose.position.z = upper_pose[2]
                pose_stamped3.pose.orientation.x = upper_quaternion[0]
                pose_stamped3.pose.orientation.y = upper_quaternion[1]
                pose_stamped3.pose.orientation.z = upper_quaternion[2]
                pose_stamped3.pose.orientation.w = upper_quaternion[3]
                go_to_pose(pose_stamped=pose_stamped3)



                # rotate
                pose2 = [x_cord,y_cord,0.1,np.pi*(90/180),np.pi*180/180,np.pi*180/180]
                quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
                # going in the shelf and release
                pose3 = [0.75,0.417, 0.1 ,np.pi*(90/180),np.pi*180/180,np.pi*180/180]
                quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
                #going back from the shelf 
                pose4 = [0.75,0.217, 0.1 ,np.pi*(90/180),np.pi*180/180,np.pi*180/180]
                quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5]) 


            

                #TODO_shikhar: coordinated such that robotdrops the wrong cube on floor
            # txt_rasa=rasa_request("Hey")
            # head_displ1ay_text(txt_rasa)
            # rospy.sleep(10)
            # txt_rasa=rasa_request("what is the error")
            # head_display_text(txt_rasa)
            # rospy.sleep(10)
            # txt_rasa=rasa_request("why the error has occured")
            # head_display_text(txt_rasa)
            
            # going in the shelf and release
            #Shikhar---- I am making change over here Please remove the comment line
            # pose3 = [0.85,-0.417, 0.45 ,np.pi*(-30/180),np.pi*180/180,np.pi*180/180]
            # quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            # #going back from the shelf 
            # pose4 = [0.85,-0.217, 0.45 ,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            # quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])            
    
    elif(data=="shelf3"):
             # rotate
            pose2 = [x_cord,y_cord,0.8,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
            # going in the shelf and release
            pose3 = [0.75,-0.417, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            #going back from the shelf 
            pose4 = [0.75,-0.217, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5]) 
            
###############################################################################
    ##############  <-- end training 
################################################################################   
    




####################### this is what should be : ################################################ 
               
# # delete the hashtags (Ctrl + ?) from here. to -->   

#     # Go to needed shelf and place
#     if (data=="shelf1_1"):
#             # rotate
#             pose2 = [x_cord,y_cord,0.15,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
#             quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
#             # going in the shelf and release
#             pose3 = [0.7,-0.417, 0.02 ,np.pi*(-20/180),np.pi*180/180,np.pi*180/180]
#             quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
#             #going back from the shelf 
#             pose4 = [0.7,-0.217, 0.02 ,np.pi*(-20/180),np.pi*180/180,np.pi*180/180]
#             quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])

    elif(data=="shelf1_2"):
            # rotate
        if (out_of_range(msg="hello",fail=1)==0):
            pose2 = [x_cord,y_cord,0.15,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
                # going in the shelf and release
            pose3 = [0.85,-0.417, 0.01 ,np.pi*(-30/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
                #going back from the shelf                             
            pose4 = [0.85,-0.217, 0.02 ,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])
        else:                
            print("FAILED")
            out_of_range(msg="hello",fail=1)
#     elif(data=="shelf2_1"):
#             # rotate
#             pose2 = [x_cord,y_cord,0.45,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
#             quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
#             # going in the shelf and release
#             pose3 = [0.7,-0.417, 0.45 ,np.pi*(-30/180),np.pi*180/180,np.pi*180/180]
#             quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
#             #going back from the shelf 
#             pose4 = [0.7,-0.217, 0.45 ,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
#             quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])
    
#     elif(data=="shelf2_2"):
#             # rotate
#             pose2 = [x_cord,y_cord,0.45,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
#             quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
#             # going in the shelf and release
#             pose3 = [0.85,-0.417, 0.45 ,np.pi*(-30/180),np.pi*180/180,np.pi*180/180]
#             quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
#             #going back from the shelf 
#             pose4 = [0.85,-0.217, 0.45 ,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
#             quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])
    
#     elif(data=="shelf3_1"):
#              # rotate
#             pose2 = [x_cord,y_cord,0.8,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
#             quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
#             # going in the shelf and release
#             pose3 = [0.7,-0.417, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
#             quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
#             #going back from the shelf 
#             pose4 = [0.7,-0.217, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
#             quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])

#     elif(data=="shelf3_2"):
#              # rotate
#             pose2 = [x_cord,y_cord,0.8,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
#             quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
#             # going in the shelf and release
#             pose3 = [0.85,-0.417, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
#             quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
#             #going back from the shelf 
#             pose4 = [0.85,-0.217, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
#             quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5]) 

#  ########## <- to here , delete hashtags   ##############################################       
#  ########################################################################################

    # pose 2 (stamped 1,2,3 is for the intial posessions)
    rospy.loginfo('Moving to the Target pose...')
    pose_stamped4 = PoseStamped()
    pose_stamped4.pose.position.x = pose2[0]
    pose_stamped4.pose.position.y = pose2[1]
    pose_stamped4.pose.position.z = pose2[2]
    pose_stamped4.pose.orientation.x = quaternion2[0]
    pose_stamped4.pose.orientation.y = quaternion2[1]
    pose_stamped4.pose.orientation.z = quaternion2[2]
    pose_stamped4.pose.orientation.w = quaternion2[3]
    go_to_pose(pose_stamped=pose_stamped4)

    # pose 3
    rospy.loginfo('Moving to the Target pose...')
    pose_stamped5 = PoseStamped()
    pose_stamped5.pose.position.x = pose3[0]
    pose_stamped5.pose.position.y = pose3[1]
    pose_stamped5.pose.position.z = pose3[2]
    pose_stamped5.pose.orientation.x = quaternion3[0]
    pose_stamped5.pose.orientation.y = quaternion3[1]
    pose_stamped5.pose.orientation.z = quaternion3[2]
    pose_stamped5.pose.orientation.w = quaternion3[3]
    go_to_pose(pose_stamped=pose_stamped5)


    # Open the gripper
    rospy.loginfo('Opening the gripper...')
    gripperChange(0)

    #pose 4
    rospy.loginfo('Moving to the Target pose...')
    pose_stamped6 = PoseStamped()
    pose_stamped6.pose.position.x = pose4[0]
    pose_stamped6.pose.position.y = pose4[1]
    pose_stamped6.pose.position.z = pose4[2]
    pose_stamped6.pose.orientation.x = quaternion4[0]
    pose_stamped6.pose.orientation.y = quaternion4[1]
    pose_stamped6.pose.orientation.z = quaternion4[2]
    pose_stamped6.pose.orientation.w = quaternion4[3]
    go_to_pose(pose_stamped=pose_stamped6)    


# def rasa_request(message):
#     API_ENDPOINT = "http://0.0.0.0:5005/webhooks/rest/webhook"
#     headers = { 'Content-type':'application/json'}
#     payload = '{"sender": "error", "message":"'+message+'"}'

#     r = requests.post(API_ENDPOINT, data=payload.encode('utf-8'), headers=headers)
#     response = json.loads(r.content)
#     return(response[0]['text'])     


def rasa_request(message):
    API_ENDPOINT = "http://0.0.0.0:5005/webhooks/rest/webhook"
    headers = { 'Content-Type': 'application/json'}
    payload = json.dumps({"sender": "error", "message": message})

    try:
        r = requests.post(API_ENDPOINT, data=payload, headers=headers)
        r.raise_for_status()  # Raises HTTPError for bad responses
        response = r.json()  # Using r.json() instead of json.loads(r.content)
        
        if not response:
            # If response list is empty, handle it
            print("Warning: Received empty response from Rasa server.")
            return "No response from Rasa server."
        
        return response[0]['text']
    
    except requests.exceptions.RequestException as e:
        # Handle any requests exceptions
        print(f"Error: {e}")
        return "Request failed."
    except (json.JSONDecodeError, KeyError, IndexError) as e:
        # Handle JSON decode errors and indexing errors
        print(f"Error: {e}")
        return "Invalid response format."


def head_display_text(txt):
     head_dip=HeadDisplay()
     fnt = ImageFont.truetype('/usr/share/fonts/truetype/freefont/FreeMono.ttf', 70)
     image = Image.new(mode = "RGB", size = (1065,600), color = "red")
     draw = ImageDraw.Draw(image)
     draw.text((100,100), txt, font=fnt, fill=(255,255,0))
     #image = Image.new(mode = "RGB", size = (200,70), color = "blue")
     image.save('yes.jpg')
     os.system('yes.jpg')
     head_dip.display_image('yes.jpg',False,30)


def fixed_x_cordination(x):
    x_fixed= -0.000003*(x**2)+0.098398*x+0.199383
    print(x_fixed)
    return ((x_fixed)/100+0.42)

def fixed_y_cordination(y):
    y_fixed=-0.0000100*(y**2)+0.1030163*y-1.5582677
    print(y_fixed)
    return ((((y_fixed)/100))*-1+0.28)


def detect_error_cube(expected_shelf,msg,fail):
     #detect_shelf=barcode_read_qr_code(0,0).data
     abc=barcode_read_qr_code(0,0)
     #ab=robot_movement()
     abc.video_reader()
     detect_shelf=abc.data
     print(detect_shelf)
     while (fail<4):
          
          if (detect_shelf != expected_shelf):
               fail+=1
               print(fail)
               swap_cube(msg)
               msg=input("Please press a to continue or ask me any questions")
               if (msg=="A" or msg=="a"):#check in python if a user press enter then how to compare
                if(fail<4):
                    print("abcd")
                    
                    #obser_vation=detect_error_cube(expected_shelf,msg,fail)
                    #print(obser_vation)
                else:
                    print("efg")
                    result=0  #########put the ascii code of the enter 
               else:
                if(fail<4):
                    print("hi")
                    swap_cube(msg)
                    #obser_vation=detect_error_cube(expected_shelf,msg,fail)
                else:
                    print("klm")
                    result=0
          else:
              result=1
              break
          abc=barcode_read_qr_code(0,0)
          #ab=robot_movement()
          abc.video_reader()
          detect_shelf=abc.data
          x_cord=fixed_x_cordination(abc.mid_x)
          y_cord=fixed_y_cordination(abc.mid_y)
          print(detect_shelf)
     if(result==1 or detect_shelf==expected_shelf):
         return 1,x_cord,y_cord
     else:
         return 0,x_cord,y_cord 
     
def out_of_range(msg,fail):
    abc=barcode_read_qr_code(0,0)
    abc.video_reader()
    x_cord=fixed_x_cordination(abc.mid_x)
    y_cord=fixed_y_cordination(abc.mid_y)
    while(fail<4):
        
        if( 121< x_cord <360 and 138 < y_cord <370):
            result=0
            break  
        else:
            fail+=1
            #put_cube_inside(msg)
            result=1
        abc=barcode_read_qr_code(0,0)
        abc.video_reader()
        x_cord=fixed_x_cordination(abc.mid_x)
        y_cord=fixed_y_cordination(abc.mid_y)
    if (result==1):
        return 1 # out of the range         
    else:
        return 0 # in the range 
    
    
def swap_cube(msg):
     txt_rasa=rasa_request(msg)
     head_display_text(txt_rasa)    
def put_cube_inside(msg):
     txt_rasa=rasa_request(msg)
     head_display_text(txt_rasa)

# Example usage
     #observation = detect_error_cube(expected_shelf="shelf3", msg="hey", fail=1)
     #print(observation)
           
                        


if __name__ == '__main__':
    main()
