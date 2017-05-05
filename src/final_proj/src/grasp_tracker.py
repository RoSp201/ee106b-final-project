#!/usr/bin/env python
import sys
import tf
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from baxter_interface import gripper as baxter_gripper
from core import transformations
import math

global ar_markers  #global
global wrist_id
global finger_id
global right_gripper

def callback(msg):
	# detect grasp if distance between tags less than grasp distance threshold
	ar_markers = {}
	found_wrist = False
	found_fingers = False
	for m in msg.markers:
		if m.id == int(wrist_id):
			found_wrist = True
			ar_markers['wrist'] = m
		elif m.id == int(finger_id):
			ar_markers['finger'] = m
			found_fingers = True

	if not found_wrist:
		print "AR tag {} was not found.".format(wrist_id)
		return
	elif not found_fingers:
		print "AR tag {} was not found.".format(finger_id)
		return



	finger_pos = np.array([[ar_markers['finger'].pose.pose.position.x, ar_markers['finger'].pose.pose.position.y, ar_markers['finger'].pose.pose.position.z]])
	wrist_pos = np.array([[ar_markers['wrist'].pose.pose.position.x, ar_markers['wrist'].pose.pose.position.y, ar_markers['wrist'].pose.pose.position.z]])
	#print "finger position: {}".format(finger_pos)
	#print "wrist position: {}".format(wrist_pos)

	# calculate distance between the tags
	dist = math.sqrt((finger_pos[0][0] - wrist_pos[0][0])**2 + (finger_pos[0][1] - wrist_pos[0][1])**2 + (finger_pos[0][2] - wrist_pos[0][2])**2)
	print "distance between finger and wrist: {}".format(dist)
	if 0 <= dist < 0.1:
		print "close gripper"
		right_gripper.close(block=True)
		rospy.sleep(0.1)
	else:
		print "open gripper"
		right_gripper.open(block=True)
		rospy.sleep(0.1)

if __name__ == '__main__':

	if len(sys.argv) < 3:
		sys.exit('Use: grasp_tracker.py [ Wrist AR tag number ] [ finger tip AR tag number ]')
	rospy.init_node("grasp_detect_node", anonymous=True)
	wrist_id = sys.argv[1]
	finger_id = sys.argv[2]
	right_gripper = baxter_gripper.Gripper('right')
	right_gripper.calibrate()
	rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
	rospy.spin()



