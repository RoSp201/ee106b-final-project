#!/usr/bin/python
# https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
import rospy
import sys
import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import baxter_pykdl as kdl
import transformations
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
    curr_pos[0] = -1*curr_pos[0]

    curr_pos[1] = -1*curr_pos[1]

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
    count = 0
    while not rospy.is_shutdown():
        # eulerr = transformations.euler_from_quaternion([0,0,1,0])
        r = np.hstack((np.array([curr_pos]),np.array([eulerr])))
        kin_left = kdl.baxter_kinematics('left')

        left_angles = left.joint_angles()
        try:
            jacobian = kin_left.jacobian()
            pinv_jacobian = kin_left.jacobian_pseudo_inverse()
            square_mtrix = np.dot(jacobian, jacobian.T)
            
        except:
            continue
        while not rospy.is_shutdown():
            try:
                t = listener.getLatestCommonTime('/base','/left_gripper')
                posl,quatl = listener.lookupTransform('/base','/left_gripper', t)
                print posl
                # posl[0] = -1*posl[0]
                # print posl
                # eulerl = transformations.euler_from_quaternion(quatl)
                eulerl = [0,0,0]
                l = np.hstack((np.array([posl]),np.array([eulerl])))
                break
            except:
                continue
                print "No Transform"
        delta_theta = np.dot(pinv_jacobian,r.T) - np.dot(pinv_jacobian,l.T)
        left.set_joint_velocities(to_dictionary(delta_theta))

if __name__ == '__main__':
    prev_pos = np.array([0,0,0])
    curr_pos = np.array([0,0,0])
    command_joint_velocities()
