#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import math
from cv_bridge import CvBridge, CvBridgeError
import math
from geometry_msgs.msg import Twist
import time


def draw_cube():
    rospy.init_node('controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10)  # 10 Hz

    move_cmd = Twist()

    # Define cube parameters
    side_length = 1.0  # meters
    speed = 1  # meters per second
    turn_speed = 1.59  # radians per second

    while not rospy.is_shutdown():      # Move forward
        move_cmd.linear.x = speed
        move_cmd.angular.z = 0
        pub.publish(move_cmd)
        time.sleep(1)

        # Turn 90 degrees
        move_cmd.linear.x = 0
        move_cmd.angular.z = turn_speed
        pub.publish(move_cmd)
        time.sleep(1)

    # Repeat similar blocks for other faces if needed

    # Stop robot after drawing
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    pub.publish(move_cmd)


if __name__ == '__main__':
    try:
        draw_cube()
    except rospy.ROSInterruptException:
        pass
