#!/usr/bin/python
# https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
import rospy
import sys
import baxter_interface
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
    curr_pos = np.array(data.data)   #data received from sub in curr pos 
    curr_pos[0] = -1*curr_pos[0]

    curr_pos[1] = -1*curr_pos[1]


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
        self.bias = 0.001 #bias term to prevent correction from being 0

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

        self.prev_integration = self.prev_integration + error

        if curr_time - self.time > 0:
            self.derivative = self.derivative / (curr_time-self.time) #multiple instead of divide because < 1
            self.prev_integration *= (curr_time-self.time)
        
        self.time = max(curr_time, self.time)
        self.prev_error = error

        return (self.kp*error) + (self.ki*self.prev_integration) + self.kd*self.derivative + self.bias


def command_joint_velocities():

    #Start a node
    rospy.init_node('baxter_joint_kinematics_node', anonymous=True)
    rospy.Subscriber("kinect_pos_track", Float32MultiArray, callback)

    #Initialize the left limb for joint velocity control
    kin_left = kdl.baxter_kinematics('left')
    kin_right = kdl.baxter_kinematics('right')

    pid = PIDController(kp=0.4, kd=0.05, ki=0.001)

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


    # get baxter pykdl jacobians
    while not rospy.is_shutdown():
        # eulerr = transformations.euler_from_quaternion([0,0,1,0])
        r = np.hstack((np.array([curr_pos]),np.array([eulerr])))
        left_angles = left.joint_angles()
        try:
            jacobian = kin_left.jacobian()
            pinv_jacobian = kin_left.jacobian_pseudo_inverse()
        except:
            continue

        # get transform for left gripper on baxter
        while not rospy.is_shutdown():
            try:
                t = listener.getLatestCommonTime('/base', '/left_gripper')
                posl, quatl = listener.lookupTransform('/base', '/left_gripper', t)
                print posl
                # posl[0] = -1*posl[0]
                # print posl
                # eulerl = transformations.euler_from_quaternion(quatl)
                euler_left_hand = [0, 0, 0]
                left_baxter_eof = np.hstack((np.array([posl]), np.array([euler_left_hand])))  #zero out orientation
                break
            except:
                continue

        delta_theta = np.dot(pinv_jacobian,r.T) - np.dot(pinv_jacobian,left_baxter_eof.T)  #take difference between positions
        left.set_joint_velocities(to_dictionary(delta_theta))

        ### old code here for reference (from merge) ###

        # right_eof_v = right.endpoint_velocity()
        # right_eof_velocity = np.array([[right_eof_v['linear'][0], right_eof_v['linear'][1], right_eof_v['linear'][2], right_eof_v['angular'][0], right_eof_v['angular'][1], right_eof_v['angular'][2]]]).T
        # new_left_vel = pinv_jacobian.dot(right_eof_velocity)

        # print "right eof velocity: \n{}".format(right_eof_velocity)
        # #print "left inverse jacobian: {}".format(pinv_jacobian)
        # #print "left joint velocities: {}".format(new_left_vel)
        # desired_joint_vels = to_dictionary(new_left_vel)
        # print "new left joint velocities: \n{}".format(desired_joint_vels)


        # actual_joint_vels = left.joint_velocities()
        # output_joint_velocities = {}

        # alpha = 0.6
        # joint_errors = {}
        # # peform pid control on the error produced by each joint on left arm
        # for key in actual_joint_vels.keys():
        #     error = desired_joint_vels[key] - actual_joint_vels[key]
        #     joint_errors[key] = error
        #     output_joint_velocities[key] = (desired_joint_vels[key]*alpha + pid.controller_output(error)*(1-alpha)) / 0.5

        # # command the left arm to move with the determined joint velocities 
        # left.set_joint_velocities(output_joint_velocities)

        # max_joint_name, max_joint_value = -1000, -1000
        # for joint_name in joint_errors.keys():
        #     if joint_errors[joint_name] > max_joint_value:
        #         max_joint_value = joint_errors[joint_name]
        #         max_joint_name = joint_name

        # # note: may want to consider adding some smoothing parameter if motions become very unstable
        # print "max joint velocity error: {} at joint: {}".format(max_joint_value, max_joint_name)
        # rospy.sleep(0.1)

if __name__ == '__main__':
    prev_pos = np.array([0,0,0])
    curr_pos = np.array([0,0,0])
    command_joint_velocities()
