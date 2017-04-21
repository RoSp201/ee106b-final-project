#!/usr/bin/env python
# this node is for finding the relative position between human chest and left hand approximate
import sys
import rospy
import numpy as np
import tf
import time
from std_msgs.msg import Float32MultiArray
import exp_quat_func as eqf

listener = None

def ar_tracker(listener, from_frame, to_frame):
    try:
        listener.waitForTransform(from_frame, to_frame, rospy.Time(0),rospy.Duration(4.0))
        pos, quat = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
        return np.array([pos[0],pos[1],pos[2]]), np.array([quat[0],quat[1],quat[2]])
    except Exception as e:
        return None, None

def human_ar_talker(ar_markers):

    human_base_frame = ar_markers['human_base']
    human_left_eof_frame = ar_markers['human_left_eof']

    pub = rospy.Publisher('kinect_pos_track', Float32MultiArray, queue_size=10)
    pub2 = rospy.Publisher('kinect_quat_track', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(5) # 20hz

    quaternion1 = None

    while not rospy.is_shutdown():

        position1, quaternion1 = ar_tracker(listener, 'camera_link', human_base_frame)
        position2, quaternion2 = ar_tracker(listener, 'camera_link', human_left_eof_frame)

        if position1 == None or quaternion1 == None or position2 == None:
            continue

        position = (position2 - position1)*2  #this is a rough scaling factor for eof diff between human and baxter, hand ar pointed left
        print position

        #human base orientation needs to be rotated about y axis -pi/2
        # make rbt for quat 1
        human_base_rbt = eqf.create_rbt(quaternion1, -np.pi/2, np.array([0,0,0]))
        print "quaternion 1: \n{}".format(human_base_rbt)

        #make rbt for hand
        human_left_eof_rbt = eqf.create_rbt(quaternion1, 1, np.array([0,0,0]))
        print "quaternion 2: \n{}".format(human_left_eof_rbt)

        #make rbt for hand
        rel_orientation = eqf.compute_gab(human_base_rbt, human_left_eof_rbt)*0.25
        print "quaternion 3: \n{}".format(eqf.find_omega_theta(rel_orientation[:3,:3])[0])




        #publish position and quaternion velocity values
        pos = Float32MultiArray()
        pos.data = position
        quat = Float32MultiArray()
        quat.data = eqf.find_omega_theta(rel_orientation[:3,:3])[0] #quaternion1
        pub.publish(pos)
        pub2.publish(quat)
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
    

