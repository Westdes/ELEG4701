#!/usr/bin/env python
# -*- coding: utf-8 -*-
# this example will publish velocity commands to topic "turtle1/cmd_vel"，message type is geometry_msgs::Twist
# set node name to velocity_publisher_{last 3 digits of you SID}
# example: sid 1155135432 and the node name will be velocity_publisher_432

import rospy
from geometry_msgs.msg import Twist
import math


import rospy
from geometry_msgs.msg import Twist
import math


def velocity_publisher():
    # ROS node initialize
    node_name = "velocity_publisher_981"
    rospy.init_node(node_name, anonymous=True)

    # Create a Publisher, queue size is 10
    msg_class = Twist
    topic_name = 'turtle1/cmd_vel'
    turtle_vel_pub = rospy.Publisher(topic_name, msg_class, queue_size=10)

    # Set loop rate
    rate = rospy.Rate(10)  # 1 Hz

    import rospy
    from geometry_msgs.msg import Twist
    import math
    import random
    
    def velocity_publisher():
        # ROS node initialize
        node_name = "velocity_publisher_981"
        rospy.init_node(node_name, anonymous=True)
        
        # Create a Publisher, queue size is 10
        msg_class = Twist
        topic_name = 'turtle1/cmd_vel'
        turtle_vel_pub = rospy.Publisher(topic_name, msg_class, queue_size=10)
        
        # Set loop rate
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            # Init geometry_msgs::Twist
            vel_msg = Twist()
            
            # Move forward with some randomness
            vel_msg.linear.x = 2.0 + random.uniform(-0.5, 0.5)  # m/s with random variation
            vel_msg.angular.z = random.uniform(-0.1, 0.1)  # rad/s with random variation
            turtle_vel_pub.publish(vel_msg)
            rospy.loginfo("Publish turtle velocity command[m/s, rad/s] , %.3f, %.3f" % (
                vel_msg.linear.x, vel_msg.angular.z))
            rospy.sleep(2)  # Move forward for 2 seconds
            
            # Stop
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            turtle_vel_pub.publish(vel_msg)
            rospy.sleep(1)
            
            # Turn 90 degrees with some randomness
            vel_msg.angular.z = (math.pi / 2) + random.uniform(-0.1, 0.1)  # rad/s with random variation
            turtle_vel_pub.publish(vel_msg)
            rospy.loginfo("Publish turtle velocity command[m/s, rad/s] , %.3f, %.3f" % (
                vel_msg.linear.x, vel_msg.angular.z))
            rospy.sleep(1)  # Turn for 1 second
            
            # Stop
            vel_msg.angular.z = 0.0
            turtle_vel_pub.publish(vel_msg)
            rospy.sleep(1)
            
            # Repeat the above steps to complete the rectangle
            for _ in range(3):
                # Move forward with some randomness
                vel_msg.linear.x = 2.0 + random.uniform(-0.5, 0.5)  # m/s with random variation
                vel_msg.angular.z = random.uniform(-0.1, 0.1)  # rad/s with random variation
                turtle_vel_pub.publish(vel_msg)
                rospy.loginfo("Publish turtle velocity command[m/s, rad/s] , %.3f, %.3f" % (
                    vel_msg.linear.x, vel_msg.angular.z))
                rospy.sleep(2)  # Move forward for 2 seconds
                
                # Stop
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                turtle_vel_pub.publish(vel_msg)
                rospy.sleep(1)
                
                # Turn 90 degrees with some randomness
                vel_msg.angular.z = (math.pi / 2) + random.uniform(-0.1, 0.1)  # rad/s with random variation
                turtle_vel_pub.publish(vel_msg)
                rospy.loginfo("Publish turtle velocity command[m/s, rad/s] , %.3f, %.3f" % (
                    vel_msg.linear.x, vel_msg.angular.z))
                rospy.sleep(1)  # Turn for 1 second
                
                # Stop
                vel_msg.angular.z = 0.0
                turtle_vel_pub.publish(vel_msg)
                rospy.sleep(1)
            
            # Delay as loop rate
            rate.sleep()
if __name__ == "__main__":
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
