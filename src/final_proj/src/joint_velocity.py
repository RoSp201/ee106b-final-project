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

def pid_controller(error, prev_error=0, prev_integration=0,Kd=0,Kp=0,Ki=0):
    try:
        integration = prev_integration + error
        derivative = (error - prev_error)
        return Kp*error + Ki*integration + Kd*derivative, error, integration
    except:
        return Kp * error, error, integration

def to_array(args):
    """take in joint angles and output them in a 7x1 matrix"""
    array = []
    jointss = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
    for i, joint in enumerate(jointss):
        array.append(args[joint])
    return np.array([array]).T

def to_dictionary(args):
    jointss = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
    ret_dict = {}
    for i, joint in enumerate(jointss):
        ret_dict[joint] = args[i][0]
    return ret_dict


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
    

    num_cmd = 80
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'right_gripper'

    right_eof_position = None


    #print velocities
    while not rospy.is_shutdown():
        found = False
        position, quaternion = None, None
        right_jacobian = kin_right.jacobian()
        right_pinv_jacobian = np.array(kin_right.jacobian_pseudo_inverse(right.joint_angles()))
        pinv_jacobian = np.array(kin_left.jacobian_pseudo_inverse(angles))

        # joint velocity control example for right arm
        # cmd = {}
        # #note this is just for demonstration, next step will be to use the transforms of each joint
        # for idx, name in enumerate(left.joint_names()):
        #     v = right.joint_velocity(right.joint_names()[idx])
        #     #if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
        #     #    v = -v
        #     cmd['left_'+name[-2:]] = v * 1.0
        # left.set_joint_velocities(cmd)
        # rospy.sleep(0.1)

        right_eof_v = right.endpoint_velocity()
        right_eof_velocity = np.array([[right_eof_v['linear'][0], right_eof_v['linear'][1], right_eof_v['linear'][2], right_eof_v['angular'][0], right_eof_v['angular'][1], right_eof_v['angular'][2]]]).T

        


        # get velocity of right end effector using Baxter PyKDL
        if right_eof_position is None:
            right_eof_position = kin_right.forward_position_kinematics(joint_values=right.joint_angles())
        print "right eof velocity: {}".format(right_eof_velocity)
        #print "right eof position: {}".format(right_eof_position)
        #print "right joint velocities: {}".format(right_velocities)
        print "left inverse jacobian: {}".format(pinv_jacobian)
        new_left_vel = pinv_jacobian.dot(right_eof_velocity)
        print "left joint velocities: {}".format(new_left_vel)
        new_left_vel = to_dictionary(new_left_vel)
        print "new left joint velocities: {}".format(new_left_vel)

        left.set_joint_velocities(new_left_vel)









        # # grab transform
        # while not found:
        #     rospy.sleep(0.5)
        #     try:
        #         t = listener.getLatestCommonTime(from_frame, to_frame)
        #         position, quaternion = listener.lookupTransform(from_frame, to_frame, t)
        #         print "\n\nposition: {}\norientation: {}\n".format(position, quaternion)
        #         found = True
        #     except Exception as e:
        #         print "Error: {}".format(e)


        # take position of end effector and transform into joint positions

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
