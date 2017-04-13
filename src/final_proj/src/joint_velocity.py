#!/usr/bin/python

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

listener = None

def command_joint_velocities():

    #Start a node
    rospy.init_node('baxter_joint_kinematics_node', anonymous=True)

    #Initialize the left limb for joint velocity control
    kin_left = kdl.baxter_kinematics('left')
    kin_right = kdl.baxter_kinematics('right')
    # limb = baxter_interface.Limb('left')
    # angles = limb.joint_angles()
    # velocities = limb.joint_velocities()
    left = kin_left._limb_interface
    right = kin_right._limb_interface

    angles = left.joint_angles()
    velocities = left.joint_velocities()
    right_velocities = right.joint_velocities()

    #jacobian = np.array(kin.jacobian(angles))
    pinv_jacobian = np.array(kin_left.jacobian_pseudo_inverse(angles))

    num_cmd = 80
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'right_gripper'

    print velocities
    while not rospy.is_shutdown():
        found = False
        position, quaternion = None, None

        cmd = {}
        #note this is just for demonstration, next step will be to use the transforms of each joint
        for idx, name in enumerate(left.joint_names()):
            v = right.joint_velocity(right.joint_names()[idx])
            #if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
            #    v = -v
            cmd['left_'+name[-2:]] = v * 1.0
        left.set_joint_velocities(cmd)
        rospy.sleep(0.1)

        # # grab transform
        # while not found:
        #     rospy.sleep(1.0)
        #     try:
        #         t = listener.getLatestCommonTime(from_frame, to_frame)
        #         position, quaternion = listener.lookupTransform(from_frame, to_frame, t)
        #         print "\n\nposition: {}\norientation: {}\n".format(position, quaternion)
        #         found = True
        #     except Exception as e:
        #         print "Error: {}".format(e)
        rospy.sleep(0.1)




        # pos = np.array(kin_left.forward_position_kinematics()).T
        # f_pos = np.array([pos[0], pos[1], pos[2],
        #                  pos[3], pos[4], pos[5], pos[6]]).reshape(7,1)

        # pos = np.array(kin_right.forward__kinematics()).T
        # print pos
        # f_pos = np.array([pos[0], pos[1], pos[2],
        #                  pos[3], pos[4], pos[5], pos[6]]).reshape(7,1)

        #print f_pos
        #print pinv_jacobian.shape

        # joint positions
        # print pinv_jacobian.T.dot(f_pos)

        # try joint velocities



        # this value gives you a matrix of the end effector's position and orientation
        #print kin_left.forward_position_kinematics()
        #print "forward velocity kinematics: {}".format(kin_left.forward_velocity_kinematics())
    

    # velocities['left_w1'] = 0.1
    # for _ in range(num_cmd):
    #     limb.set_joint_velocities(velocities)
    #     #print "forward velocity kinematics: {}".format(kin.forward_velocity_kinematics())
    #     print velocities
    #     time.sleep(0.1)
    # time.sleep(0.5)

    # velocities['left_e1'] = -0.1
    # for _ in range(num_cmd):
    #     limb.set_joint_velocities(velocities)
    #     time.sleep(0.1)
    # time.sleep(0.5)


if __name__ == '__main__':
    command_joint_velocities()
