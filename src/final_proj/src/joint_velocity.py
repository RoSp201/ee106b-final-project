#!/usr/bin/python

import rospy
import sys
import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import baxter_pykdl as kdl
#import transformations.py
import numpy as np

import tf
import time

listener = None

def command_joint_velocities():

    #Start a node
    rospy.init_node('baxter_joint_kinematics_node', anonymous=True)

    #Initialize the left limb for joint velocity control
    kin = kdl.baxter_kinematics('left')
    #limb = baxter_interface.Limb('left')
    # angles = limb.joint_angles()
    # velocities = limb.joint_velocities()
    limb = kin._limb_interface
    angles = limb.joint_angles()
    velocities = limb.joint_velocities()

    #jacobian = np.array(kin.jacobian(angles))
    pinv_jacobian = np.array(kin.jacobian_pseudo_inverse(angles))

    num_cmd = 80
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'right_gripper'

    found = False
    position, quaternion = None, None

    # grab transform
    while not found:
        rospy.sleep(1.0)
        try:
            t = listener.getLatestCommonTime(from_frame, to_frame)
            position, quaternion = listener.lookupTransform(from_frame, to_frame, t)
            print "\n\nposition: {}\n orientation: {}\n".format(position, quaternion)
            found = True
        except Exception as e:
            print "Error: {}".format(e)



    #print velocities
    print "\n\n"

    # this value gives you a matrix of the end effector's position and orientation
    print kin.forward_position_kinematics()
    print "forward velocity kinematics: {}".format(kin.forward_velocity_kinematics())
    



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