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
class robot_movement ():
    def __init__(self):
        self.limb = Limb()
        self.traj_options= TrajectoryOptions()
        self.traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        self.traj = MotionTrajectory(trajectory_options = self.traj_options, limb = self.limb)
        self.wpt_opts = MotionWaypointOptions(max_linear_speed=0.5,
                                         max_linear_accel=0.5,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=1.0)
        self.waypoint = MotionWaypoint(options = self.wpt_opts.to_msg(), limb = self.limb)
        self.joint_names = self.limb.joint_names()
	    #print(joint_names)
        self.tip_name='right_hand'
        self.joint_angles = self.limb.joint_ordered_angles()
        self.endpoint_state = self.limb.tip_state(self.tip_name)
    def gotopos(self,pos):
        pose = self.endpoint_state.pose
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = pos[3]
        pose.orientation.y = pos[4]
        pose.orientation.z = pos[5]
        pose.orientation.w = pos[6]
        print(pose)
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
	    #print(pose_stamped)
        self.waypoint.set_cartesian_pose(pose_stamped, self.tip_name, self.joint_angles)

        rospy.loginfo('Sending waypoint: \n%s', self.waypoint.to_string())
	    #print(waypoint)
        self.traj.append_waypoint(self.waypoint.to_msg())	
        result = self.traj.send_trajectory(timeout=None)
	    #rospy.logerr(result)
        if result is None:
            rospy.logerr('Trajectory FAILED to send')

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    def qr_code_recog(self):
        cam = cv2.VideoCapture(0)
        detector = cv2.QRCodeDetector()
        while True:
            _, img = cam.read()
            data, bbox, _ = detector.detectAndDecode(img)
            if data:
                print("QR Code detected-->", data)
                return(data)
                break
            cv2.imshow("img", img)    
            if cv2.waitKey(1) == ord("Q"):
                break
        cam.release()
        cv2.destroyAllWindows()
def main():
    rospy.init_node('go_to_joint_angles_py')
    ab=robot_movement()
    data=ab.qr_code_recog()
    if (data=="shelf1"):
        ab.gotopos([0.7,0.0,0.1,0.0,1.0,0.0,0.0])
    elif(data=="shelf2"):
        ab.gotopos([0.7,0.0,0.2,0.0,1.0,0.0,0.0])
    elif(data=="shelf3"):
        ab.gotopos([0.7,0.0,0.3,0.0,1.0,0.0,0.0])
if __name__=="__main__":
    main()