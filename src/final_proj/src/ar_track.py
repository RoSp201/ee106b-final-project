#!/usr/bin/env python
# this node is for finding the relative position between human chest and left hand approximate
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
br = None

# .81 x .01 y .15 z virt cam

def ar_tracker(listener, from_frame, to_frame):
    try:
        listener.waitForTransform(from_frame, to_frame, rospy.Time(0),rospy.Duration(1.0))
        pos, quat = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
        #print "quat {}".format(quat)

        # rot = transformations.quaternion_about_axis(np.pi, [0,0,1])

        # quat = transformations.quaternion_multiply(rot, quat)


        #rot = transformations.quaternion_about_axis(np.pi, [0,1,0])

        #quat = transformations.quaternion_multiply(rot, [quat[3],quat[0],quat[1],quat[2]])



        return np.array([pos[0],pos[1],pos[2]]), quat

    except Exception as e:
        return None, None

def human_ar_talker(ar_markers):

    human_base_frame = ar_markers['human_base']
    human_left_eof_frame = ar_markers['human_left_eof']

    pub = rospy.Publisher('kinect_pos_track', Float32MultiArray, queue_size=100)
    pub2 = rospy.Publisher('kinect_quat_track', Float32MultiArray, queue_size=100)

    rate = rospy.Rate(10.0) # 10hz

    # set once as translation of virtual camera frame with respect to baxter's base
    chest_pos = None

    while not rospy.is_shutdown():


        position1, _ = ar_tracker(listener, 'camera_link', human_base_frame)


        # want to make virtual frame a fixed translation (chest position seen by kinect) in baxter base frame
        #if chest_pos != None:
        #    br.sendTransform((chest_pos[0], chest_pos[1], chest_pos[2]), (0,0,0,1), rospy.Time.now(), "camera_link", "base")

        position2, quaternion2 = ar_tracker(listener, 'base', 'ar_marker_0')

        position = None
        if position1 == None or position2 == None or quaternion2 == None:
            continue

        position = (position2 - position1)*2  #this is a rough scaling factor for eof diff between human and baxter, hand ar pointed left
        print "position: {}".format(position)

        # rot = transformations.quaternion_about_axis(-np.pi/2, [0,1,0])


        # quaternion2 = transformations.quaternion_multiply(rot, quaternion2)
        #quaternion1 = transformations.slerp_quaternion(quaternion1, quaternion2, 1)
        print "quaterion 2: \n", quaternion2

        #publish position and quaternion velocity values
        pos = Float32MultiArray()
        pos.data = position
        quat = Float32MultiArray()
        quat.data = quaternion2
        pub.publish(pos)
        pub2.publish(quat)
        rate.sleep()


if __name__ == '__main__':

    #Start a nodes
    rospy.init_node('human_ar_talker', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster() # used to add frame to tf tree connected to Baxter

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
    

