#!/usr/bin/python


#import copy
#import rospy
#import sys

import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import baxter_pykdl
import transformations

#import IPython
import tf
import time

def main():

	print("here")
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Start tf node
    # listener = tf.TransformListener()
    # from_frame = 'base'
    # to_frame = 'ar_marker_0'
    # time.sleep(1)
    # if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
    #     print 'Frames not found'
    #     print 'Did you place AR marker 0 within view of the baxter left hand camera?'
    #     exit(0)

    #Initialize the left limb for joint velocity control
    limb = baxter_interface.Limb('left')
    angles = limb.joint_angles()
    velocities = limb.joint_velocities()
    
    #Command joint velocites
    num_cmd = 25
    cmd_velocities = copy.deepcopy(velocities)

    print 'Commanding velocity'
    cmd_velocities['left_e1'] = 0.1
    for i in range(num_cmd):
        limb.set_joint_velocities(cmd_velocities)
        time.sleep(0.1)
    time.sleep(0.5)

    cmd_velocities['left_e1'] = -0.1
    for i in range(num_cmd):
        limb.set_joint_velocities(cmd_velocities)
        time.sleep(0.1)
    time.sleep(0.5)

    raw_input("pause for next move\n")
    #Command joint torques
    cmd_torques = copy.deepcopy(velocities)

    print 'Commanding torque'
    cmd_torques['left_e1'] = 0.1
    for i in range(num_cmd):
        limb.set_joint_torques(cmd_torques)
        time.sleep(0.1)
    time.sleep(0.5)

    cmd_torques['left_e1'] = -0.1
    for i in range(num_cmd):
        limb.set_joint_torques(cmd_torques)
        time.sleep(0.1)
    time.sleep(0.5)

    #Initialize the moveit commander for the left arm
    print 'Setting up planning'
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)

    # #Get pose
    # t = listener.getLatestCommonTime(from_frame, to_frame)
    # position, quaternion = listener.lookupTransform(from_frame, to_frame, t)

    #First goal pose ------------------------------------------------------
    # goal_1 = PoseStamped()
    # goal_1.header.frame_id = "base"

    # #x, y, and z position
    # goal_1.pose.position.x = position[0]
    # goal_1.pose.position.y = position[1]
    # goal_1.pose.position.z = position[2] + 0.1
    
    # #Orientation as a quaternion
    # goal_1.pose.orientation.x = 0.0
    # goal_1.pose.orientation.y = 1.0
    # goal_1.pose.orientation.z = 0.0
    # goal_1.pose.orientation.w = 0.0

    # #Set the goal state to the pose you just defined
    # left_arm.set_pose_target(goal_1)

    # #Set the start state for the left arm
    # left_arm.set_start_state_to_current_state()

    # #Plan a path
    # left_plan = left_arm.plan()

    # #Execute the plan
    # raw_input('Press <Enter> to move the left arm to the AR marker: ')
    # left_arm.execute(left_plan)

if __name__ == '__main__':
    main()