#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import math
import numpy as np

from geometry_msgs.msg import Pose2D


# Header header            # timestamp in the header is the acquisition time of 
#                          # the first ray in the scan.
#                          #
#                          # in frame frame_id, angles are measured around 
#                          # the positive Z axis (counterclockwise, if Z is up)
#                          # with zero angle being forward along the x axis
						 
# float32 angle_min        # start angle of the scan [rad]
# float32 angle_max        # end angle of the scan [rad]
# float32 angle_increment  # angular distance between measurements [rad]

# float32 time_increment   # time between measurements [seconds] - if your scanner
#                          # is moving, this will be used in interpolating position
#                          # of 3d points
# float32 scan_time        # time between scans [seconds]

# float32 range_min        # minimum range value [m]
# float32 range_max        # maximum range value [m]

# float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
# float32[] intensities    # intensity data [device-specific units].  If your
#                          # device does not provide intensities, please leave
#                          # the array empty.

from nav_msgs.msg import Odometry

laserScan = LaserScan()  
laserScan.header.frame_id = "laser" 
laserScan.angle_min = -3.14
laserScan.angle_max = 3.13
laserScan.range_min = 0
laserScan.range_max = 10
laserScan.angle_increment = 0.01

ballPose_x = 0.0
ballPose_y = 0.0
ballPose_status = False



def laserScan_cal(y, x):
	global laserScan
	ang = math.atan2(y,x)
	ang = round(ang,2)
	size_output = int((laserScan.angle_max - laserScan.angle_min)*100 + 1)
	output_scan = np.empty(size_output,float) 

	output_scan[:] = 10.0

	dist_ball = math.sqrt(math.pow(x,2) + math.pow(y,2))
	rospy.loginfo("dist_ball")
	rospy.loginfo(dist_ball)

	radius = .10 

	buff = math.atan2(radius,dist_ball)
	buff = abs(buff)

	buff = round(buff,2)
	start = int((ang - buff - laserScan.angle_min)/laserScan.angle_increment)
	end = int((ang + buff  - laserScan.angle_min)/laserScan.angle_increment) +1

	# modulus to make array circular 
	start = start % size_output
	end = end % size_output

	if end < start:
		output_scan[start:size_output] = dist_ball
		output_scan[0:end+1] = dist_ball
	else:
		output_scan[start:end+1] = dist_ball


	return output_scan

def ball_pose_callback(ball_pose_msg):
	rospy.loginfo("here")
	global ballPose_status
	global ballPose_x
	global ballPose_y
	ballPose_x = ball_pose_msg.x
	ballPose_y = ball_pose_msg.y
	
	if ballPose_status == False:
		ballPose_status = True

def odom_callback(odom_data):
	global ballPose_status
	global ballPose_x
	global ballPose_y

	seq = 1
	prevTime = 0
	 

	if ballPose_status:
		laserScan.header.seq = seq
		laserScan.header.stamp =  odom_data.header.stamp
		
		rospy.loginfo("Time " )
		rospy.loginfo(laserScan.header.stamp)
		# rospy.loginfo(laserScan.header.stamp.nsecs )

		laserScan.ranges = laserScan_cal(ballPose_x,ballPose_y)
		seq += 1
		# prevTime = laserScan.scan_time
		fakescan_pub.publish(laserScan)



if __name__ == '__main__':
	try:

		rospy.init_node('fake_laser', log_level=rospy.DEBUG)
		fakescan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
		ball_pose_sub = rospy.Subscriber('/ball_pose_pub', Pose2D, ball_pose_callback,  queue_size=10)
		odom_sub = rospy.Subscriber("/odom",Odometry, odom_callback,  queue_size = 1)

		rospy.spin()
	except rospy.ROSInterruptException:
		pass
