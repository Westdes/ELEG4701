#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import math
from cv_bridge import CvBridge, CvBridgeError
import math
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    # initialize a node named 'controller'
    rospy.init_node('controller')
    # define a publisher publishing Twist, named pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # define a transform listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # claim a Twist message
    vel = Twist()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # lookup the transform of aruco w.r.t. bask_link
        trans = tfBuffer.lookup_transform(
            'aruco', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        print(trans.transform.translation.x, trans.transform.translation.y)
        marker_pose = trans.transform.translation
        # a simple proportional controller

        # TODO: compute the linear velocity and angular velocity for the turtlebot
        # Please refer to lab4 reference code follower.py  mypose_callback function
        # Note: You need to difine linear and angular velocity (total of 6)
        # Note: You need to use marker_pose.x & .y
        kP_linear = 0.5
        kP_angular = 2.0
        vel.linear.x = kP_linear * marker_pose.x

        # Line2 for linear.y
        vel.linear.y = 0

        # Line3 for linear.z
        vel.linear.z = 0

        # Line4 for angular.x
        vel.angular.x = 0

        # Line5 for angular.y
        vel.angular.y = 0

        # Line6 for angular.z
        vel.angular.z = kP_angular * math.atan2(marker_pose.y, marker_pose.x)

        ##

        # TODO: publish the velocity with pub
        pub.publish(vel)
        ##

        rate.sleep()
