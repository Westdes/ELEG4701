#!/usr/bin/python
# rospy for the subscriber
import rospy
from sensor_msgs.msg import Image
# TODO 1: import ROS Image message
# Ref: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html


# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# import sys for the args
import sys

# Instantiate CvBridge
bridge = CvBridge()


def image_callback(msg):
    print("Received an image!")
    global count
    file_name = f"/home/edward/cakin_ws/src/lab7/imgs/image_{count}.jpeg"
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # TODO 2: Save your OpenCV2 image (cv2_img) as a png/jpeg
        print(f"writing image in to {file_name}")
        cv2.imwrite(file_name, cv2_img)
        count += 1


def main():
    # TODO 3: Init the node
    image_listener = rospy.init_node('image_listener')
    # TODO 4: Set up your subscriber and subscribe the color image
    image_sub = rospy.Subscriber(
        "/camera/color/image_raw", Image, image_callback)

    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    count = 1
    main()
