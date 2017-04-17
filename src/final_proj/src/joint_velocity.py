#!/usr/bin/python
# https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
import rospy
import sys
import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import baxter_pykdl as kdl
#import transformations.py   <-- there was an issue importing while running
import numpy as np

import tf
import time
from std_msgs.msg import Float32MultiArray
import ar_track

listener = None

prev_pos = np.array([0,0,0])
curr_pos = np.array([0,0,0])

def to_array(args):
    array = []
    jointss = args.keys()
    for i,joint in enumerate(jointss):
        array.append(args[joint])
    return np.array([array]).T

def to_dictionary(args):
    jointss = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
    ret_dict = {}
    for i,joint in enumerate(jointss):
        ret_dict[joint] = args[i]
    return ret_dict

def callback(data):
    global curr_pos
    curr_pos = np.array(data.data)

def command_joint_velocities():

    #Start a node
    rospy.init_node('baxter_joint_kinematics_node', anonymous=True)
    rospy.Subscriber("kinect_pos_track", Float32MultiArray, callback)

    #Initialize the left limb for joint velocity control
    kin_left = kdl.baxter_kinematics('left')
    kin_right = kdl.baxter_kinematics('right')

    left = kin_left._limb_interface
    right = kin_right._limb_interface

    listener = tf.TransformListener()

    right_angles = right.joint_angles()
    while not rospy.is_shutdown():
        try:
            t = listener.getLatestCommonTime('/base','/right_gripper')
            posr,quatr = listener.lookupTransform('/base','/right_gripper', t)
            eulerr = [0,0,0]
            r = np.hstack((np.array([posr]),np.array([eulerr])))
            break
        except:
            continue
    r = np.hstack((-1*np.array([curr_pos]),np.array([eulerr])))
    # print velocities
    while not rospy.is_shutdown():

        r = np.hstack((np.array([curr_pos][::-1]),np.array([eulerr])))
        kin_left = kdl.baxter_kinematics('left')

        left_angles = left.joint_angles()

        pinv_jacobian = kin_left.jacobian_pseudo_inverse()
        print(pinv_jacobian.shape)
        # print(kin_left.jacobian(left.joint_angles()).shape)
        # print(np.array([[0,0,0,0,0,0]]).shape)
        while not rospy.is_shutdown():
            try:
                t = listener.getLatestCommonTime('/base','/left_gripper')
                posl,quatl = listener.lookupTransform('/base','/left_gripper', t)
                eulerl = [0,0,0]
                l = np.hstack((np.array([posl]),np.array([eulerl])))
                break
            except:
                continue
        delta_theta = np.dot(pinv_jacobian,(r-l).T)
        #note this is just for demonstration, next step will be to use the transforms of each joint
        # print(delta_theta)
        left.set_joint_velocities(to_dictionary(delta_theta*.4))

if __name__ == '__main__':
    prev_pos = np.array([0,0,0])
    curr_pos = np.array([0,0,0])
    command_joint_velocities()
