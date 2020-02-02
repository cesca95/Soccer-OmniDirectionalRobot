#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import math
import numpy as np


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

laserScan = LaserScan()  
laserScan.header.frame_id = "map" 
laserScan.angle_min = -1.57
laserScan.angle_max = 1.56
laserScan.range_min = 0
laserScan.range_max = 10
laserScan.angle_increment = 0.01

def laserScan_cal(y, x):
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
    end = int(ang + buff  - laserScan.angle_min/laserScan.angle_increment) +1

    rospy.loginfo("here1")
    rospy.loginfo(start)
    rospy.loginfo(end)

    theta = -1*(ang - buff)
    while start <= end:
        output_scan[start] = dist_ball
        theta = theta + laserScan.angle_increment
        start += 1
        # rospy.loginfo(output_scan[start])
        # rospy.loginfo(start)
        # rospy.loginfo(end)
        # rospy.loginfo("here inside")

    return output_scan



def talker():
    fakescan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    seq = 1
    prevTime = 0

    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        
        laserScan.header.seq = seq
        laserScan.header.stamp = rospy.get_rostime()
        
        laserScan.scan_time = laserScan.header.stamp.secs - prevTime

        laserScan.ranges = laserScan_cal(0,1.5)
        seq += 1
        prevTime = laserScan.scan_time
        fakescan_pub.publish(laserScan)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
