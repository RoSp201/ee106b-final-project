#!/usr/bin/env python
import sys
import rospy
import numpy as np
import tf
import time
from std_msgs.msg import Float32MultiArray
import exp_quat_func as eqf
from core import transformations
import geometry_msgs.msg

listener = None
# number of sample points want to take median of
NUM = 10  

def ar_tracker(listener, from_frame, to_frame):
    try:
        listener.waitForTransform(from_frame, to_frame, rospy.Time(0),rospy.Duration(1.0))
        pos, quat = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
        return np.array([pos[0],pos[1],pos[2]]), np.array([quat[0], quat[1], quat[2], quat[3]])
    except Exception as e:
        return None, None

def filter(positions):
    """
    Applies a median filter to positions received from the subscriber before commanding a move by baxter
    """
    positions.sort(key=lambda l:l[0])
    positions.sort(key=lambda l:l[1])
    positions.sort(key=lambda l:l[2])

    # take median of these values
    median = positions[len(positions) / 2]
    return median


def human_ar_talker(ar_markers):

    human_base_frame = ar_markers['human_base']
    human_left_eof_frame = ar_markers['human_left_eof']

    pub = rospy.Publisher('kinect_pos_track', Float32MultiArray, queue_size=30)
    pub2 = rospy.Publisher('kinect_quat_track', Float32MultiArray, queue_size=30)
    rate = rospy.Rate(100.0) 

    i = 0
    position_buff = [None]*NUM
    filter_position = None

    while not rospy.is_shutdown():

        position1, _ = ar_tracker(listener, '/camera_link', human_base_frame)
        position2, quaternion2 = ar_tracker(listener, '/camera_link', human_left_eof_frame)

        if position1 == None or position2 == None or quaternion2 == None:
            continue

        # scaled each dimension proportional to human full extension
        position = (position2 - position1)
        position[0] *= 1.3
        position[1] *= 1.6
        position[2] *= 1.45
 
        if i == NUM:
            filter_position = filter(position_buff)
            pos = Float32MultiArray()
            pos.data = filter_position
            quat = Float32MultiArray()
            quat.data = quaternion2
            pub.publish(pos)
            pub2.publish(quat)
            i = 0

        else:
            position_buff[i] = position
            i += 1
        rate.sleep()


if __name__ == '__main__':

    #Start a nodes
    rospy.init_node('human_ar_talker', anonymous=True)
    listener = tf.TransformListener()

    if len(sys.argv) < 2:
        print 'Use: rosrun final_proj ar_track.py [ chest AR tag number ] [ left hand AR tag number ]'
        sys.exit()

    ar_markers = {}
    ar_markers['human_base'] = 'ar_marker_' + sys.argv[1]
    ar_markers['human_left_eof'] = 'ar_marker_' + sys.argv[2]
    try:
        human_ar_talker(ar_markers)
    except rospy.ROSInterruptException:
        pass
    

