#!/usr/bin/env python  

import rospy
import tf
import math
from core import transformations
import numpy as np

if __name__ == '__main__':

	rospy.init_node('my_tf_broadcaster')
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():

		rot_z = transformations.quaternion_about_axis(math.pi, [0,0,1])
		t = rospy.Time.now().to_sec() * math.pi
		br.sendTransform((1.2, 0.01, 0.15),(rot_z[0], rot_z[1], rot_z[2], rot_z[3]), rospy.Time.now(), "camera_link", "base")
		rate.sleep()
