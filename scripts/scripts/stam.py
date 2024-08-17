#! /usr/bin/env python
"""A program to move the item from pos1 to pos2 using the gripper function"""

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
from geometry_msgs.msg import PoseStamped
from gripper import gripperChange  # Assuming you have the gripperChange function implemented in the gripper module

def main():
    rospy.init_node('move_item_once')
    limb = Limb()
    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj = MotionTrajectory(trajectory_options=traj_options, limb=limb)
    wpt_opts = MotionWaypointOptions(max_linear_speed=0.5,
                                     max_linear_accel=0.5,
                                     max_rotational_speed=1.57,
                                     max_rotational_accel=1.57,
                                     max_joint_speed_ratio=1.0)

    # Initial pose for picking up the item
    #pos_pick = [0.7,0.0,0.3,0.0,1.0,0.0]
    pos_pick = [0.75, -0.2, -0.03,np.pi*180/180,np.pi*1/180,np.pi*180/180]
    quaternion_pick = tf.transformations.quaternion_from_euler(pos_pick[3], pos_pick[4], pos_pick[5])
    tip_name = 'right_hand'
    joint_angles = limb.joint_ordered_angles()

    # Target pose for releasing the item
    pos_release = [0.676,0.46,0.116,0.0,np.pi*90/180,np.pi*180/180]
    quaternion_release = tf.transformations.quaternion_from_euler(pos_release[3], pos_release[4], pos_release[5])

    # Set initial pose for picking up
    initial_pose_stamped = PoseStamped()
    initial_pose_stamped.pose.position.x = pos_pick[0]
    initial_pose_stamped.pose.position.y = pos_pick[1]
    initial_pose_stamped.pose.position.z = pos_pick[2]
    initial_pose_stamped.pose.orientation.x = quaternion_pick[0]
    initial_pose_stamped.pose.orientation.y = quaternion_pick[1]
    initial_pose_stamped.pose.orientation.z = quaternion_pick[2]
    initial_pose_stamped.pose.orientation.w = quaternion_pick[3]
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
    waypoint.set_cartesian_pose(initial_pose_stamped, tip_name, joint_angles)
    traj.append_waypoint(waypoint.to_msg())

    rospy.loginfo('Closing the gripper...')
    gripperChange(1)

    rospy.loginfo('Sending move to pos1 trajectory...')
    result = traj.send_trajectory(timeout=None)

    if result is None:
        rospy.logerr('Trajectory FAILED to send')

    if result.result:
        rospy.loginfo('Motion controller successfully reached pos1!')

        # Open the gripper for releasing after reaching pos1
        rospy.loginfo('Opening the gripper...')
        gripperChange(0)

        # Set target pose for releasing (pos2)
        target_pose_stamped = PoseStamped()
        target_pose_stamped.pose.position.x = pos_release[0]
        target_pose_stamped.pose.position.y = pos_release[1]
        target_pose_stamped.pose.position.z = pos_release[2]
        target_pose_stamped.pose.orientation.x = quaternion_release[0]
        target_pose_stamped.pose.orientation.y = quaternion_release[1]
        target_pose_stamped.pose.orientation.z = quaternion_release[2]
        target_pose_stamped.pose.orientation.w = quaternion_release[3]
        waypoint.set_cartesian_pose(target_pose_stamped, tip_name, joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        rospy.loginfo('Closing the gripper...')
        gripperChange(1)

        rospy.loginfo('Sending move to pos2 trajectory...')
        result = traj.send_trajectory(timeout=None)

        if result is None:
            rospy.logerr('Trajectory FAILED to send')

        if result.result:
            rospy.loginfo('Motion controller successfully reached pos2!')

            # Open the gripper for releasing after reaching pos2
            rospy.loginfo('Opening the gripper...')
            gripperChange(0)

    else:
        rospy.logerr('Motion controller failed to complete the move to pos1 trajectory with error %s',
                     result.errorId)

if __name__ == '__main__':
    main()



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
from geometry_msgs.msg import PoseStamped
from gripper import gripperChange  # we have the gripperChange function implemented in the gripper module

def main():
    rospy.init_node('perform_sequence')
    limb = Limb()
    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj = MotionTrajectory(trajectory_options=traj_options, limb=limb)
    wpt_opts = MotionWaypointOptions(max_linear_speed=0.5,
                                     max_linear_accel=0.5,
                                     max_rotational_speed=1.57,
                                     max_rotational_accel=1.57,
                                     max_joint_speed_ratio=1.0)

    # Pose 1
    pose1 = [0.75, -0.2, -0.03,np.pi*160/180,np.pi*1/180,np.pi*160/180]
    quaternion1 = tf.transformations.quaternion_from_euler(pose1[3], pose1[4], pose1[5])

    # Pose 2
    pose2 = [0.676, 0.46, 0.116, 0.0, np.pi*90/180, np.pi*180/180]
    quaternion2 = tf.transformations.quaternion_from_euler(pose2[3], pose2[4], pose2[5])

    tip_name = 'right_hand'
    joint_angles = limb.joint_ordered_angles()

    # Open the gripper
    rospy.loginfo('Opening the gripper...')
    gripperChange(1)
    time.sleep(1)

    # Go to Pose 1
    rospy.loginfo('Moving to Pose 1...')
    pose_stamped1 = PoseStamped()
    pose_stamped1.pose.position.x = pose1[0]
    pose_stamped1.pose.position.y = pose1[1]
    pose_stamped1.pose.position.z = pose1[2]
    pose_stamped1.pose.orientation.x = quaternion1[0]
    pose_stamped1.pose.orientation.y = quaternion1[1]
    pose_stamped1.pose.orientation.z = quaternion1[2]
    pose_stamped1.pose.orientation.w = quaternion1[3]
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
    waypoint.set_cartesian_pose(pose_stamped1, tip_name, joint_angles)
    traj.append_waypoint(waypoint.to_msg())
    result = traj.send_trajectory(timeout=None)


    if result is None or not result.result:
        rospy.logerr('Failed to move to Pose 1. Error: %s', result.errorId)
        return
    
    # Close the gripper
    rospy.loginfo('Closing the gripper...')
    gripperChange(1)
    time.sleep(1)

    # Go to Pose 2
    rospy.loginfo('Moving to Pose 2...')
    pose_stamped2 = PoseStamped()
    pose_stamped2.pose.position.x = pose2[0]
    pose_stamped2.pose.position.y = pose2[1]
    pose_stamped2.pose.position.z = pose2[2]
    pose_stamped2.pose.orientation.x = quaternion2[0]
    pose_stamped2.pose.orientation.y = quaternion2[1]
    pose_stamped2.pose.orientation.z = quaternion2[2]
    pose_stamped2.pose.orientation.w = quaternion2[3]
    waypoint.set_cartesian_pose(pose_stamped2, tip_name, joint_angles)
    traj.append_waypoint(waypoint.to_msg())
    result = traj.send_trajectory(timeout=None)

    if result is None or not result.result:
        rospy.logerr('Failed to move to Pose 2')
        return

    # Open the gripper
    rospy.loginfo('Opening the gripper...')
    gripperChange(0)

if __name__ == '__main__':
    main()
