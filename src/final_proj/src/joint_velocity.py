#!/usr/bin/python
# https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
import rospy
import sys
import baxter_interface
import baxter_pykdl as kdl
import numpy as np
from numpy.linalg import svd
import tf
import time
from std_msgs.msg import Float32MultiArray
import ar_track
from core import transformations

listener = None
curr_pos = np.array([0,0,0])
curr_rot = np.array([0,0,0])

def to_array(args, limb='right'):
    array = []
    jointss = ['_s0','_s1','_e0','_e1','_w0','_w1','_w2']
    jointss = [limb+joint for joint in jointss]  # allows for use of either right or left limb

    for i,joint in enumerate(jointss):
        array.append(args[joint])
    return np.array([array]).T

def to_dictionary(args, limb='right'):
    args = np.array(args).ravel()
    jointss = ['_s0','_s1','_e0','_e1','_w0','_w1','_w2']
    jointss = [limb+joint for joint in jointss]  # allows for use of either right or left limb

    ret_dict = {}
    for i,joint in enumerate(jointss):
        ret_dict[joint] = args[i]

    return ret_dict

def nullspace(A, atol=1e-3,rtol=0):
    A = np.atleast_2d(A)
    u,s,vh = svd(A)
    tol = max(atol,rtol*s[0])
    nnz = (s>=tol).sum()
    ns = vh[nnz:].conj().T
    return ns

def callback(data):
    global curr_pos
    curr_pos = np.array(data.data)   #data received from sub in curr pos 
    curr_pos[0] = -1*curr_pos[0]
    curr_pos[1] = -1*curr_pos[1]

def callback1(data):
    global curr_rot
    curr_quat = data.data
    curr_rot = transformations.euler_from_quaternion(curr_quat)


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
    # angles for desired position
    # desired_angle_dict = {'right_s0': 0.05062136600021865, 'right_s1': -1.0028399400800891, 'right_w0': -0.6665146523362123, 'right_w1': 1.0147282911862012, 'right_w2': 0.5276893910325823, 'right_e0': 1.20992734644462, 'right_e1': 1.9493060862053895}
    desired_angle_dict = {'right_s0': 0.6507913492603867, 'right_s1': -0.6243301806693634, 'right_w0': -0.6711165946998685, 'right_w1': 1.4599662148699426, 'right_w2': -0.08360195293975504, 'right_e0': 0.8444564237309202, 'right_e1': 0.7205874751091731}
    desired_angles_array = to_array(desired_angle_dict)

    rospy.init_node('baxter_joint_kinematics_node', anonymous=True)
    rospy.Subscriber("kinect_pos_track", Float32MultiArray, callback)
    rospy.Subscriber("kinect_quat_track", Float32MultiArray, callback1)

    #Initialize the left limb for joint velocity control
    kin_left = kdl.baxter_kinematics('left')
    kin_right = kdl.baxter_kinematics('right')

    pid = PIDController(kp=0.4, kd=0.05, ki=0.001)

    #left = kin_left._limb_interface
    right = kin_right._limb_interface

    listener = tf.TransformListener()
    right_angles = right.joint_angles()

    while not rospy.is_shutdown():
        euler_human_hand = [0,0,0]
        r = np.hstack((np.array([curr_pos]),np.array([euler_human_hand])))
        #left_angles = left.joint_angles()
        right_angles = right.joint_angles() #change for update right
        # print(right_angles)
        try:
            #jacobian = kin_left.jacobian()
            jacobian = kin_right.jacobian()

            # calulate nullspace of jacobian
            null_jacob = np.array(nullspace(jacobian))

            # diff between current and desired angles (gradient)
            # print(right_angles)
            grad_angles = -to_array(right_angles)+desired_angles_array
            x = np.array(np.dot(null_jacob,null_jacob.T))
            join_angle_adjustments = np.dot(x,grad_angles)

            # pinv_jacobian = kin_left.jacobian_pseudo_inverse()
            pinv_jacobian = kin_right.jacobian_pseudo_inverse() #update

        except:

            continue

        while not rospy.is_shutdown():
            try:
                # t = listener.getLatestCommonTime('/base', '/left_gripper')
                # posl, quatl = listener.lookupTransform('/base', '/left_gripper', t)
                t = listener.getLatestCommonTime('/base', '/right_gripper')
                posr, quatr = listener.lookupTransform('/base', '/right_gripper', t)
                eulerr = [0,0,0]
                # left_baxter_eof = np.hstack((np.array([posr]), np.array([eulerr])))
                right_baxter_eof = np.hstack((np.array([posr]), np.array([eulerr])))
                break
            except Exception as e:
                print "Lookup Error: {}".format(e)
                continue

        # delta_theta = np.dot(pinv_jacobian,r.T) - np.dot(pinv_jacobian, left_baxter_eof.T)  #take difference between joint angles
        # joint_v = to_dictionary(delta_theta)
        # left.set_joint_velocities(joint_v)
        delta_theta = np.dot(pinv_jacobian, r.T) - np.dot(pinv_jacobian, right_baxter_eof.T)  #take difference between joint angles
        
        # add joint_angle_adjustments to delta_theta
        delta_theta = delta_theta + join_angle_adjustments
        joint_v = to_dictionary(delta_theta, 'right')
        #left.set_joint_velocities(joint_v)
    
        joint_v['right_w2'] = .08*curr_rot[2]
        print .08*curr_rot[2]
        joint_v['right_w1'] *= -1
        right.set_joint_velocities(joint_v)
        
if __name__ == '__main__':
    curr_pos = np.array([0,0,0])
    command_joint_velocities()