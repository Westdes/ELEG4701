#!/usr/bin/python
# rospy for the subscriber
import rospy

# TODO 1: import ROS LaserScan message,
# Ref: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html

from sensor_msgs.msg import LaserScan


# TODO 2: the callback function

def callbackFunction(msg):
    print("Received an laser scan!")
    ranges = msg.ranges
    average = sum(ranges) / len(ranges)
    rospy.loginfo(average)


def main():
    # TODO 3: init the ros_node, the subscirber, and rospy.spin()
    rospy.init_node('task_IV_subscriber')
    image_sub = rospy.Subscriber(
        "/scan", LaserScan, callbackFunction)

    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
