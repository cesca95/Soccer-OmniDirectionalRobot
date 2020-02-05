#!/usr/bin/env python

# Author: Astha Gupta

# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

from robot_control.msg import RobotVision
from robot_control.srv import BallPose, BallPoseResponse
from nav_msgs.msg import Odometry
import tf
# from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D



import math

class ball_pose:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.odom_sub = rospy.Subscriber("odom",
            Odometry, self.callback_odom,  queue_size = 1)

        self.camera_x = 0.125
        self.camera_y = 0
        
        self.ball_srv = rospy.Service('/ball_pose_srv', BallPose, self.handle_ball_pose)
        self.ballpose_pub = rospy.Publisher('/ball_pose_pub', Pose2D,  queue_size=10)

        self.odom_pose = Pose2D()
        self.ball_pose = Pose2D()

        self.ball_dist = 0

        self.odom_status = False
        self.ballpose_cal = False
        self.distnew = False

    def handle_ball_pose(self,req):
        # print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))

        rospy.loginfo("handle_ball_pose")
        self.ball_dist = req.dist
        while(self.ballpose_cal == False):
            print("working")
            # keep stalling 
        resp = BallPoseResponse()
        resp.x = self.ball_pose.x
        resp.y = self.ball_pose.x
        resp.status = True
        self.distnew = True
        return resp

    def callback_odom(self, odom_data):
        rospy.loginfo("callback_odom")
        self.odom_pose.x = odom_data.pose.pose.position.x
        self.odom_pose.y = odom_data.pose.pose.position.y
        quaternion = odom_data.pose.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        euler = tf.transformations.euler_from_quaternion(explicit_quat)

        rospy.loginfo(euler[2])
        self.odom_pose.theta = euler[2]

        rospy.loginfo(" ang " )
        rospy.loginfo(self.odom_pose.theta )

        if self.odom_status == False:
            self.odom_status = True

        if self.ball_dist != 0 and self.odom_status and self.distnew:
            # calculate the Pose2D
            theta_right = -1*self.odom_pose.theta
            self.ball_pose.x = self.odom_pose.x  + math.cos(theta_right)*(self.camera_x + self.ball_dist)
            self.ball_pose.y = self.odom_pose.y  + math.sin(theta_right)*(self.camera_y + self.ball_dist)
            self.ball_pose.theta = 0

            rospy.loginfo(self.ball_pose.x)
            rospy.loginfo(self.ball_pose.y)

            self.ballpose_pub.publish(self.ball_pose)
            self.ballpose_cal  = True
            self.distnew = False
        else:
            self.ballpose_pub.publish(self.ball_pose)



def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('ball_pose', log_level=rospy.DEBUG)
    ic = ball_pose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
