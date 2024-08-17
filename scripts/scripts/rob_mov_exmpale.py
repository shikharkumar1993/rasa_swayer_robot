#! /usr/bin/env python
"""A program to move the robot from pos A to pos B"""

import rospy
import multiprocessing
#from libezgripper import create_connection, Gripper
import socket
import time
from std_msgs.msg import Int32, String
import tf
#from qr_code_reader import data
import numpy as np
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
import geometry_msgs
def main():
        rospy.init_node('go_to_joint_angles_py')
        limb = Limb()
        traj_options= TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
        wpt_opts = MotionWaypointOptions(max_linear_speed=0.5,
                                         max_linear_accel=0.5,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
        joint_names = limb.joint_names()
	#print(joint_names)
        #pos=[0.7,0.0,0.3,0.0,1.0,0.0,0.0]
        #pos=[0.7,0.0,0.25,np.pi*0/180,np.pi*180/180,np.pi*180/180]
        pos=[0.7,0.0,0.40,np.pi*10/180,np.pi*180/180,np.pi*180/180]
        quaternion = tf.transformations.quaternion_from_euler(pos[3], pos[4], pos[5])
        print("Adam",quaternion)
        tip_name='right_hand'
        joint_angles = limb.joint_ordered_angles()

      
	
        "The above three line is to initialize the robot"
        endpoint_state = limb.tip_state(tip_name)
	#print(endpoint_state)
        pose = endpoint_state.pose
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
	#print(pose_stamped)
        waypoint.set_cartesian_pose(pose_stamped, tip_name, joint_angles)

        rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())
	#print(waypoint)
        traj.append_waypoint(waypoint.to_msg())	
        result = traj.send_trajectory(timeout=None)
	#rospy.logerr(result)
        if result is None:
            rospy.logerr('Trajectory FAILED to send')

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
	
if __name__ == '__main__':
    main()

