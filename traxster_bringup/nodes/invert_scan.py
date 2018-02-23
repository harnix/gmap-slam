#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class InvertScan():
    def __init__(self):
        rospy.init_node('invert_scan',anonymous=True) 
        rospy.on_shutdown(self.shutdown)
        self.scan_pub = rospy.Publisher('base_scan', LaserScan, queue_size=50)
        rate = rospy.Rate(.5)
        while not rospy.is_shutdown():
            self.sub=rospy.Subscriber('/scan',LaserScan,self.scan_invert)

            rate.sleep()    

    def scan_invert(self,scan):
        current_time = rospy.Time.now()

        new_scan = LaserScan()

        new_scan.header.stamp = scan.header.stamp
        new_scan.header.frame_id = scan.header.frame_id
        new_scan.angle_min = scan.angle_min
        new_scan.angle_max = scan.angle_max
        new_scan.angle_increment = scan.angle_increment
        new_scan.time_increment = scan.time_increment
        new_scan.range_min = scan.range_min
        new_scan.range_max = scan.range_max

        new_scan.ranges = scan.ranges[::-1]
        new_scan.intensities = scan.intensities[::-1]
        #print(new_scan.ranges)

        self.scan_pub.publish(new_scan)

    def shutdown(self): 
        pass

if __name__ == '__main__':
    try:
        InvertScan()
    except Exception as e:
        rospy.loginfo(e)