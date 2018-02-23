#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class OdometryPublisher():
    def __init__(self):

        self.SVL = 0
        self.SVR = 0
        rospy.init_node('odometry_publisher')

        rospy.Subscriber('/ard_odom', Twist, self.ard_odom_callback)

        odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        odom_broadcaster = tf.TransformBroadcaster()



        x = 0.0
        y = 0.0
        th = 0.0

        v = 0
        w = 0

        wheelSeparation = 0.6

        #values from 0.95 - 1.05
        odomTurnMultiplier = 0.95


        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():

            current_time = rospy.Time.now()

            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).to_sec()
            DL = dt * self.SVL
            DR = dt * self.SVR

            dxy = (DL + DR) / 2
            dth = (((DR - DL) / wheelSeparation))*odomTurnMultiplier

            x += dxy * cos(th)
            y += dxy * sin(th)
            th += dth

            v = dxy/dt
            w = dth/dt

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

            # publish the message
            odom_pub.publish(odom)

            last_time = current_time
        r.sleep()

    def ard_odom_callback(self, data):
        self.SVL = data.linear.x
        self.SVR = data.linear.y

if __name__ == '__main__':
    try:
        OdometryPublisher()
    except Exception as e:
        rospy.loginfo(e)



    