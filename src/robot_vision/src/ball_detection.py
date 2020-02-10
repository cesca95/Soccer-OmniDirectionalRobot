#!/usr/bin/env python

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

import math
res_width = 320
res_height = 240
foallength_pixels = res_width/(2*math.tan(math.radians(62.2)/2));

VERBOSE=True

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.info_pub = rospy.Publisher("/robot_vision",
            RobotVision, queue_size=1)

        # subscribed Topic
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw/compressed",
            CompressedImage, self.callback_image,  queue_size = 1)


        if VERBOSE :
            print "subscribed to /rrbot/camera1/image_raw"



    def callback_image(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        redUpper1 = (10, 255, 255)
        redLower1 = (0, 70, 50)
        redUpper2 = (180, 255, 255)
        redLower2 = (170, 70, 50)
        

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, redLower1, redUpper1)
        mask2 = cv2.inRange(hsv, redLower2, redUpper2)
        mask = mask1|mask2
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found

        # for the control part of the project
        control_info = RobotVision()
        control_info.Ball = False
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # print((x,y))

            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # control_info.GoalX = center[0]
            # control_info.GoalY = center[1]
            # print(center)

            dist = -1
            # only proceed if the radius meets a minimum size
            if radius > 5:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                dist = (foallength_pixels*(10))/(radius)

            control_info.BallCenterX = center[0]
            control_info.BallCenterY = center[1]
            control_info.Ball  = 1
            control_info.BallRadius = np.uint8(int(radius))
            control_info.DistBall = dist*(0.01)
            # update the points queue
            #pts.appendleft(center)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)
            control_info.Ball = True
        self.info_pub.publish(control_info)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('ball_detection', anonymous=True)
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
