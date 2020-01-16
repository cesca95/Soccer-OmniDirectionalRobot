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

VERBOSE=True

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/rrbot/camera1/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /rrbot/camera1/image_raw"


    def callback(self, ros_data):
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
	#cv2.imshow('mask', mask)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
		if radius > 5:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(image_np, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(image_np, center, 5, (0, 0, 255), -1)
			# width(640)/(2*tan(62.2/2))
			foallength_pixels = 530.469971363
			dist = (foallength_pixels*(10*100))/(radius)
			print dist
 
	# update the points queue
	#pts.appendleft(center)
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

      
        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
