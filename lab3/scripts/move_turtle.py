#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class turtle_control:
    def __init__(self, publisher_name, subscriber_name) -> None:
        # TODO 1: initialize publisher and subscriber
        self.pub = rospy.Publisher(publisher_name, Twist, queue_size=10)
        self.sub = rospy.Subscriber(
            subscriber_name, Pose, self.turtle_control_callback)

    def turtle_control_callback(self, pose: Pose):
        rospy.loginfo(
            "Turtle1's pose: [%.2f, %.2f, %.2f]", pose.x, pose.y, pose.theta)

        # TODO 2: calculate velocity commands to draw a circle with linear velocity of 3.14 m/s and radius of 3.14 m
        # when the x position is larger than 8, stop the turtle
        vel_cmd = Twist()
        linear_velocity = 3.14
        radius = 3.14
        angular_velocity = linear_velocity / radius

        if pose.x >= 8:
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
            rospy.loginfo('Robot exercises finished. Stopping robot.')
        else:
            vel_cmd.linear.x = linear_velocity
            vel_cmd.angular.z = angular_velocity

        rospy.loginfo(
            "Publishing velocity command [%.2fm/s, %.2frad/s]",
            vel_cmd.linear.x,
            vel_cmd.angular.z,
        )

        # TODO 3: publish velocity commands
        self.pub.publish(vel_cmd)


def move_turtle():
    rospy.init_node("move_turtle", anonymous=False)
    turtle_control("turtle1/cmd_vel", "turtle1/pose")
    rospy.spin()


if __name__ == "__main__":
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass
