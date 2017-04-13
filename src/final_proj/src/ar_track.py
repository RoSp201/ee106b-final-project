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

def ar_tracker(listener,from_fram,to_frame):
    try:
        listener.waitForTransform(from_frame, to_frame, rospy.Time(0),rospy.Duration(4.0))
        pos, quat = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
        return pos, quat
    except Exception as e:
        print e
        return None, None

if __name__ == '__main__':
    ar_tags = ['ar_marker_1', 'ar_marker_6']

    #Start a node
    rospy.init_node('moveit_node')

    #Start tf node
    listener = tf.TransformListener()
    from_frame = ar_tags[0]
    to_frame = ar_tags[1]

    while not rospy.is_shutdown():
        position, quaternion = ar_tracker(listener,from_frame,to_frame)
        if position == None or quaternion == None:
            continue
        print [position, quaternion]


