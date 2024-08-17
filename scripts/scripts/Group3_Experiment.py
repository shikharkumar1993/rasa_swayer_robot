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

import socket
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
def initial_pick(x_cord,y_cord):
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


def main():
   rospy.init_node('perform_sequence')
   out_range=0
   swap_error=0
   head_display_text(" ")
   for i in range(0,12): #need 7
    
        
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
    abc.detect_number_times_error_range(out_range)
    data=abc.data
    x_cord=fixed_x_cordination(abc.mid_x)
    y_cord=fixed_y_cordination(abc.mid_y)
    print(x_cord)
    print(y_cord)
    print(data)

    

#########################################################################################
    ############## this is for training , because the printed and stacked qr codes on the cubes temporary now
    ################## the data in the qr we will print will be like this : 
    ##################   shelf1_1 , shelf1_2 , shelf1_3 
    ##################   shelf2_1 , shelf2_2 , shelf2_3
    ##################   shelf3_1 , shelf3_2 , shelf3_3
#########################################################################################
     
    
                

    # Go to needed shelf and place
    if (data=="shelf1_1"):
            initial_pick(x_cord=x_cord,y_cord=y_cord)
            # rotate
            pose2 = [x_cord,y_cord,0.15,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
            # going in the shelf and release
            pose3 = [0.7,-0.417, 0.02 ,np.pi*(-20/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            #going back from the shelf 
            pose4 = [0.7,-0.217, 0.02 ,np.pi*(-20/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])

### ERR 1 "OUT OF RANGE" ###
            
    elif(data=="shelf1_2"):
        if(out_range!=0):
            continue   
            # rotate
        out_range+=1
        obser2,x_cordi,y_cordi=out_of_range(msg="hello",fail=1)
        print(obser2)
        if (obser2==0):
            initial_pick(x_cord=x_cordi,y_cord=y_cordi)
            pose2 = [x_cordi,y_cordi,0.15,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
                # going in the shelf and release
            pose3 = [0.85,-0.4, 0.01 ,np.pi*(-30/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
                #going back from the shelf                             
            pose4 = [0.85,-0.2, 0.01 ,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])
        else:                
            print("FAILED")
            #out_of_range(msg="hello",fail=1)
            continue
        head_display_text(" ")

### ERR Swap cube -> shelf2
    elif(data=="shelf2_1"):
            # rotate
            #TODO_GABRIEL: put the cube down
            if(swap_error!=0):
                continue
            swap_error+=1
            initial_pick(x_cord=x_cord,y_cord=y_cord)
            down_pose=[x_cord,y_cord,-0.02,np.pi*0/180,np.pi*180/180,np.pi*180/180]
            down_quaternion = tf.transformations.quaternion_from_euler(down_pose[3], down_pose[4], down_pose[5])

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
            down_quaternion = tf.transformations.quaternion_from_euler(down_pose[3], down_pose[4], down_pose[5])

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


            obser_vation,x_cordi,y_cordi=detect_error_cube(expected_shelf="shelf2_3",msg="hey",fail=1)
            print(obser_vation)
            print("++++++++++++++++++++++++")

            if(obser_vation==1):
                # print("SUCCESS")
                # pose2 = [x_cord,y_cord,0.45,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
                # quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
                initial_pick(x_cord=x_cordi,y_cord=y_cordi)
                # rotate
                pose2 = [x_cordi,y_cordi,0.47,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
                quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
                # going in the shelf and release
                pose3 = [0.75,-0.417, 0.47 ,np.pi*(-30/180),np.pi*180/180,np.pi*180/180]
                quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
                #going back from the shelf 
                pose4 = [0.75,-0.217, 0.47 ,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
                quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])


            elif(obser_vation==0 ):
                print("FAILED")
                #continue

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



                pose2 = [x_cord,y_cord,0.15,np.pi*(40/180),np.pi*180/180,np.pi*180/180]
                quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
                # going in the shelf and release
                pose3 = [0.6,0.417, 0.01 ,np.pi*(20/180),np.pi*180/180,np.pi*180/180]
                quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
                #going back from the shelf 
                pose4 = [0.6,+0.217, 0.01 ,np.pi*(20/180),np.pi*180/180,np.pi*180/180]
                quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])
            head_display_text(" ")

    elif(data=="shelf2_3"):
            initial_pick(x_cord=x_cord,y_cord=y_cord)
            # rotate
            pose2 = [x_cord,y_cord,0.47,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
            # going in the shelf and release
            pose3 = [0.75,-0.417, 0.47 ,np.pi*(-30/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            #going back from the shelf 
            pose4 = [0.75,-0.217, 0.47 ,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])

    elif(data=="shelf2_2"):
            initial_pick(x_cord=x_cord,y_cord=y_cord)
            # rotate
            pose2 = [x_cord,y_cord,0.47,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
            # going in the shelf and release
            pose3 = [0.85,-0.417, 0.47 ,np.pi*(-30/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            #going back from the shelf 
            pose4 = [0.85,-0.217, 0.47 ,np.pi*(-40/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])
    
    elif(data=="shelf3_1"):
            initial_pick(x_cord=x_cord,y_cord=y_cord)
             # rotate
            pose2 = [x_cord,y_cord,0.8,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
            # going in the shelf and release
            pose3 = [0.6,-0.417, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            #going back from the shelf 
            pose4 = [0.6,-0.217, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])

    elif(data=="shelf3_2"):
            initial_pick(x_cord=x_cord,y_cord=y_cord)
             # rotate
            pose2 = [x_cord,y_cord,0.8,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])
            # going in the shelf and release
            pose3 = [0.75,-0.417, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion3 = tf.transformations.quaternion_from_euler(pose3[3], pose3[4], pose3[5])
            #going back from the shelf 
            pose4 = [0.75,-0.217, 0.8 ,np.pi*(-90/180),np.pi*180/180,np.pi*180/180]
            quaternion4 = tf.transformations.quaternion_from_euler(pose4[3], pose4[4], pose4[5])

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


def rasa_request(message,sender_id):
    API_ENDPOINT = "http://0.0.0.0:5005/webhooks/rest/webhook"
    headers = { 'Content-Type': 'application/json'}
    payload = json.dumps({"sender": sender_id, "message": message})

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
def det_cube_swap():
     detect_shelf=barcode_read_qr_code(0,0).data
     abc=barcode_read_qr_code(0,0)
     #ab=robot_movement()
     abc.error_cubes()
     x_cord=fixed_x_cordination(abc.mid_x)
     y_cord=fixed_y_cordination(abc.mid_y)
     detect_shelf=abc.data
     print("@@@@@")
     print(detect_shelf)
     return detect_shelf,x_cord,y_cord,abc.mid_x,abc.mid_y


def detect_error_cube(expected_shelf,msg,fail):
     detect_shelf,x_cord,y_cord,mid_x,mid_y=det_cube_swap()
     #while (fail<4):
     print("$$$$$$$$$$$$$$$$$$$$$$")
     print(detect_shelf)          
          
          
          
     if (detect_shelf!= expected_shelf):
               fail+=1
               print(fail)
               msg_rcvd=swap_cube(msg)
               msg=msg_rcvd
               #msg=input("Please press a to continue or ask me any questions")
               if not msg:#check in python if a user press enter then how to compare
                if(fail<4):
                    print("abcd")
                    result=0
                    
                    #obser_vation=detect_error_cube(expected_shelf,msg,fail)
                    #print(obser_vation)
                else:
                    print("efg")
                    result=0  #########put the ascii code of the enter
                    #x_cord=x_co
                    #y_cord=y_co
                     
               else:
                if(fail<4):
                    print("hi")
                    result=0
                    swap_cube(msg)
                    
                    #obser_vation=detect_error_cube(expected_shelf,msg,fail)
                else:
                    print("klm")
                    #x_cord=x_co
                    #y_cord=y_co
                    result=0
                    
     else:
              result=1              
              #break
     detect_shelf,x_cord,y_cord,mid_x,mid_y=det_cube_swap()
     if (detect_shelf== expected_shelf):
          result=1
     if (fail==4):
         detect_shelf,x_cord,y_cord,mid_x,mid_y=det_cube_swap()

     if(result==1 or detect_shelf==expected_shelf):
         return 1,x_cord,y_cord
     else:
         return 0,x_cord,y_cord 
def det_out_range():
     detect_shelf=barcode_read_qr_code(0,0).data
     abc=barcode_read_qr_code(0,0)
     #ab=robot_movement()
     abc.error_cubes_out_range()
     x_cord=fixed_x_cordination(abc.mid_x)
     y_cord=fixed_y_cordination(abc.mid_y)
     detect_shelf=abc.data
     print("@@@@@")
     print(detect_shelf)
     return detect_shelf,x_cord,y_cord,abc.mid_x,abc.mid_y    
def out_of_range(msg,fail):
    data,x_cord,y_cord,mid_x,mid_y=det_out_range()
    #x_min=fixed_x_cordination(121)
    #x_max=fixed_x_cordination(360)
    #y_min=fixed_y_cordination(138)
    #y_max=fixed_y_cordination(370)
    in_msg=put_cube_inside("hi")
    while(fail<3):
        
        
        data,x_cord,y_cord,mid_x,mid_y=det_out_range()        
        if( (121<mid_x <360) and (138 < mid_y <370)):
            result=0
            print("in_range_cube:success")
            break  
        else:
            #ADDRASA:SHIKHAR
            #in_msg=input("Please press a to continue or ask me any questions")
            #if (in_msg!='a' or in_msg != 'A'):
            msg=put_cube_inside(in_msg)
            in_msg=msg
            fail+=1
            #put_cube_inside(msg)
            result=1
        data,x_cord,y_cord,mid_x,mid_y=det_out_range()
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print(mid_x,mid_y)
        
    if (fail==4):
        data,x_cord,y_cord,mid_x,mid_y=det_out_range()
        print("$$$$$$$$$$$",data)
    if (result==0 or ((121<mid_x <360) and (138 < mid_y <370))):
            return 0,x_cord,y_cord # in the range         
    else:
        return 1,x_cord,y_cord # out of the range 



def communication(message_rasa):
    TCP_IP='132.72.96.201'
    TCP_PORT=5006
    BUFFER_SIZE=1024

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    #
    #print ('Connected by', addr)
    data_decoded=""
    conn, addr = s.accept()
    conn.send(message_rasa.encode())
    while True:
         print('123')
         data = conn.recv(BUFFER_SIZE)
         
         #print(data.decode())
         if not data:                     
            break
         
         print ("received data:", data.decode())
         if data:
            data_decoded=data.decode()
         # echo
    print("@@@@@@@@@@@",data_decoded)
    conn.close()    
    return(data_decoded)
  
def swap_cube(msg):
     #abc=rasa_request("I am E1G2","E1G2")         
     #txt_rasa=rasa_request(msg,"E1G2")
    #  msg=communication(txt_rasa)
    #  print("#########",msg) 
     txt_rasa="Error, I’m unable to put the item on shelf."
     txt_split=txt_rasa.split()
     txt_new=[]
     for i in range(0,len(txt_split),3):
          txt_new.append(' '.join(txt_split[i:i+3]))
     txt='\n'.join(txt_new)
     head_display_text(txt)
     msg=communication(txt_rasa)
     print("#########",msg) 
     return msg 
   
def put_cube_inside(msg):
     abc=rasa_request("I am E2G2","E2G2")
     txt_rasa=rasa_request(msg,"E2G2")
     #msg=communication(txt_rasa)
     txt_split=txt_rasa.split()
     txt_new=[]
     for i in range(0,len(txt_split),3):
          txt_new.append(' '.join(txt_split[i:i+3]))
     txt='\n'.join(txt_new)
     head_display_text(txt)
     msg=communication(txt_rasa)
     return msg

# Example usage
     #observation = detect_error_cube(expected_shelf="shelf3", msg="hey", fail=1)
     #print(observation)
           
                        


if __name__ == '__main__':
    main()
