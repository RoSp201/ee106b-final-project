#!/usr/bin/python
# https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
import rospy
import sys
import baxter_interface
import baxter_pykdl as kdl
import numpy as np
import tf
import time
from std_msgs.msg import Float32MultiArray
import ar_track
from core import transformations

listener = None
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

def callback1(data):
    global curr_rot
    curr_rot = data.data


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
    rospy.init_node('baxter_joint_kinematics_node', anonymous=True)
    rospy.Subscriber("kinect_pos_track", Float32MultiArray, callback)
    rospy.Subscriber("kinect_quat_track", Float32MultiArray, callback1)

    #Initialize the left limb for joint velocity control
    kin_left = kdl.baxter_kinematics('left')
    kin_right = kdl.baxter_kinematics('right')

    pid = PIDController(kp=0.4, kd=0.05, ki=0.001)

    left = kin_left._limb_interface
    right = kin_right._limb_interface

    listener = tf.TransformListener()
    right_angles = right.joint_angles()

    while not rospy.is_shutdown():
        euler_human_hand = [0,0,0]
        r = np.hstack((np.array([curr_pos]),np.array([euler_human_hand])))
        left_angles = left.joint_angles()
        try:
            jacobian = kin_left.jacobian()
            pinv_jacobian = kin_left.jacobian_pseudo_inverse()
        except:
            continue

        while not rospy.is_shutdown():
            try:
                t = listener.getLatestCommonTime('/base', '/left_gripper')
                posl, quatl = listener.lookupTransform('/base', '/left_gripper', t)
                eulerl = [0,0,0]
                left_baxter_eof = np.hstack((np.array([posl]), np.array([eulerl])))
                break
            except Exception as e:
                print "ERROR: {}".format(e)
                continue

        delta_theta = np.dot(pinv_jacobian,r.T) - np.dot(pinv_jacobian, left_baxter_eof.T)  #take difference between joint angles
        joint_v = to_dictionary(delta_theta)
        left.set_joint_velocities(joint_v)
        
if __name__ == '__main__':
    curr_pos = np.array([0,0,0])
    command_joint_velocities()
