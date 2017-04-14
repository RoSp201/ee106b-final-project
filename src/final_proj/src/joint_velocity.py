#!/usr/bin/python

# Joint Velocity Controller
# Robert Spark, Vidush Mukund, Gabriel Al-Harbi
# UC Berkeley, Spring 2017

import rospy
import sys
import baxter_interface
import baxter_pykdl as kdl
#import transformations.py   <-- there was an issue importing while running
import numpy as np
import tf
import time

listener = None

def pid_controller(error, prev_error=0, prev_integration=0, Kd=0.0, Kp=0.0, Ki=0.0):

    try:
        integration = prev_integration + error
        derivative = (error - prev_error)
        return Kp*error + Ki*integration + Kd*derivative, error, integration
    except:
        return Kp * error, error, integration

class PIDController(object):

    def __init__(self, error=0, prev_error=0, prev_integration=0, kd=0.0, kp=0.0, ki=0.0, time=0.0):
        self.kd = kd
        self.kp = kp
        self.ki = ki
        self.prev_error = prev_error
        self.error = error
        self.prev_integration = prev_integration
        self.time = time
        self.derivative = 0

    def __str__(self):
        return "PID Controller: \n  Kp: {}\n  Kd: {}\n  Ki: {}\n  Error: {}\n  Time: {}".format(self.kp, self.kd, self.ki, self.error, self.time)

    def set_kp(self, kp=0.0):
        self.kp = kp

    def set_ki(self, ki=0.0):
        self.ki = ki

    def set_kd(self, kd=0.0):
        self.kd = kd

    def controller_output(self, error, curr_time=0):
        """
        using the given pid gains, apply the correctional output with the most recent error
        """
        self.derivative = (error - self.prev_error)

        if curr_time - self.time > 0:
            self.derivative /= (curr_time-self.time)

        self.prev_integration = self.prev_integration + error
        self.time = max(curr_time, self.time)
        self.prev_error = error

        return Kp*self.prev_error + Ki*self.prev_integration + Kd*self.derivative


def to_array(args):
    """take in joint angles and output them in a 7x1 matrix."""
    array = []
    jointss = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
    for i, joint in enumerate(jointss):
        array.append(args[joint])
    return np.array([array]).T


def to_dictionary(args):
    """
    This function takes in an 7x1 np array and assigns the velocities to the corresponding joints on the baxter arm
    """
    jointss = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
    ret_dict = {}
    for i, joint in enumerate(jointss):
        ret_dict[joint] = args[i][0]
    return ret_dict


def command_joint_velocities():

    # Start a node
    rospy.init_node('baxter_joint_kinematics_node', anonymous=True)

    # Initialize the left limb for joint velocity control
    kin_left = kdl.baxter_kinematics('left')
    kin_right = kdl.baxter_kinematics('right')
    left = kin_left._limb_interface
    right = kin_right._limb_interface

    left_angles = left.joint_angles()
    right_angles = None
    left_velocities = left.joint_velocities()
    right_velocities = right.joint_velocities()
    jacobian = np.array(kin.jacobian(angles))
    
    num_cmd = 80
    listener = tf.TransformListener()

    desired_joint_vels = left.joint_velocities()

    from_frame = 'base'
    to_frame = 'right_gripper'

    right_eof_position = None

    # create new pid controller instance
    pid = PIDController()
    print pid

    while not rospy.is_shutdown():

        found = False
        right_angles = right.joint_angles()
        position, quaternion = None, None
        right_jacobian = kin_right.jacobian()
        right_pinv_jacobian = np.array(kin_right.jacobian_pseudo_inverse(right_angles))
        pinv_jacobian = np.array(kin_left.jacobian_pseudo_inverse(left_angles))


        ### TODO: Use transform from model to grab correct transform ###

        # while not found:
        #     rospy.sleep(0.5)
        #     try:
        #         t = listener.getLatestCommonTime(from_frame, to_frame)
        #         position, quaternion = listener.lookupTransform(from_frame, to_frame, t)
        #         print "\n\nposition: {}\norientation: {}\n".format(position, quaternion)
        #         found = True
        #     except Exception as e:
        #         print "Error: {}".format(e)



        #### Joint velocity control example for right arm ###

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
        new_left_vel = pinv_jacobian.dot(right_eof_velocity)

        print "right eof velocity: \n{}".format(right_eof_velocity)
        #print "left inverse jacobian: {}".format(pinv_jacobian)
        #print "left joint velocities: {}".format(new_left_vel)
        desired_joint_vels = to_dictionary(new_left_vel)
        print "new left joint velocities: \n{}".format(desired_joint_vels)

        # command the left arm to move with the determined joint velocities 
        #left.set_joint_velocities(new_left_vel)

        # TODO: add some form of control to this to make these movements more exact
        actual_joint_vels = left.joint_velocities()
        output_joint_velocities = {}

        for key in actual_joint_vels.keys():
            error = desired_joint_vels[key] - actual_join_vels[key]
            output_joint_velocities[key] = desired_joint_vels[key] + pid.controller_output(error)

        # command the left arm to move with the determined joint velocities 
        #left.set_joint_velocities(output_joint_velocities)

        # note: may want to consider adding some smoothing parameter if motions become very unstable
        print "joint velocity error: {}".format(error)
        rospy.sleep(0.1)

   

if __name__ == '__main__':
    command_joint_velocities()




