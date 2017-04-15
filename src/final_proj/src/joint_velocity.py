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
from std_msgs.msg import Float32MultiArray
import ar_track

listener = None

prev_pos = np.array([0,0,0])
curr_pos = np.array([0,0,0])

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

    # print velocities
    while not rospy.is_shutdown():
        found = False
        position, quaternion = None, None

        cmd = {}
        #note this is just for demonstration, next step will be to use the transforms of each joint
        for idx, name in enumerate(left.joint_names()):
            # if name[-2:] == 's1':
            v = right.joint_velocity(right.joint_names()[idx])
            #if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
            #    v = -v
            # need to convert velocity to rad/s
            cmd['left_'+name[-2:]] = v
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
    prev_pos = np.array([0,0,0])
    curr_pos = np.array([0,0,0])
    command_joint_velocities()
