#!/usr/bin/env python
"""
Starter script for EE106B grasp planning lab
Author: Jeff Mahler
"""
import warnings

import sys
import rospy
import numpy as np
import tf
import time
from core import RigidTransform, Point, transformations as t
from meshpy import ObjFile
from visualization import Visualizer3D as vis

import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
import baxter_interface
from std_msgs.msg import Float32MultiArray

def ar_tracker(listener,from_frame,to_frame):
    try:
        listener.waitForTransform(from_frame, to_frame, rospy.Time(0),rospy.Duration(4.0))
        pos, quat = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
        return np.array([pos[0],pos[1],pos[2]]), np.array([quat[0],quat[1],quat[2]])
    except Exception as e:
        # print e
        return None, None

def talker():

    rospy.init_node('talker', anonymous=True)
    ar_tags = ['ar_marker_1', 'ar_marker_4']

    #Start tf node
    listener = tf.TransformListener()
    from_frame = ar_tags[0]
    to_frame = ar_tags[1]

    pub = rospy.Publisher('kinect_pos_track', Float32MultiArray, queue_size=10)

    pub2 = rospy.Publisher('kinect_quat_track', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        position, quaternion = ar_tracker(listener,from_frame,to_frame)

        if position == None or quaternion == None:
            continue
        #publish position and quaternion velocity values
        pos = Float32MultiArray()
        pos.data = position
        quat = Float32MultiArray()
        quat.data = quaternion
        pub.publish(pos)
        pub2.publish(quat)
        rate.sleep()



if __name__ == '__main__':
    #Start a node
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    

