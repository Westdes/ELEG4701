#!/usr/bin/env python
# -*- coding: utf-8 -*-
# this example will publish velocity commands to topic "turtle1/cmd_vel"，message type is geometry_msgs::Twist
# set node name to velocity_publisher_{last 3 digits of you SID}
# example: sid 1155135432 and the node name will be velocity_publisher_432

import rospy
from geometry_msgs.msg import Twist
import math


def velocity_publisher():
    # TODO 1, ROS node initialize
    node_name = "velocity_publisher_981"
    rospy.init_node(node_name, anonymous=True)
    # create a Publisher, queue size is 10
    # TODO 2: finish the code below
    msg_class = Twist
    topic_name = 'turtle1/cmd_vel'
    # reference: rospy.Publisher(topic_name, msg_class, queue_size)
    turtle_vel_pub = rospy.Publisher(topic_name, msg_class, queue_size=10)
    # set loop rate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # init geometry_msgs::Twist
        vel_msg = Twist()
        # TODO 3: draw a circle, the linear velocity is π m/s, and radius is π m
        # modify the code below and import something at the start of the file, you should use the π in math library rather than 3.14
        vel_msg.linear.x = math.pi
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 1
        # publish
        turtle_vel_pub.publish(vel_msg)
        # TODO 4: modify the code below to display velocity commands, keep 3 decimal places
        rospy.loginfo(
            "Publish turtle velocity command[m/s, rad/s] , %.3f, %.3f" % (
                vel_msg.linear.x, vel_msg.angular.z)
        )
        # delay as loop rate
        rate.sleep()


if __name__ == "__main__":
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
