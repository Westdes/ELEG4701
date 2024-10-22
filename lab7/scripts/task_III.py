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

# Instantiate CvBridge
bridge = CvBridge()


# TODO 2: define your own callback function for color image
def callbackFunction_I(msg):
    print("Received an colour image!")
    global count
    file_name = f"/home/edward/cakin_ws/src/lab7/imgs/colour_image_{count}.jpeg"
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


# TODO 3: define your own callback function for depth image
def callbackFunction_II(msg):
    print("Received an depth image!")
    global count
    file_name = f"/home/edward/cakin_ws/src/lab7/imgs/depth_image_{count}.jpeg"
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "16UC1")
    except CvBridgeError as e:
        print(e)
    else:
        # TODO 2: Save your OpenCV2 image (cv2_img) as a png/jpeg
        print(f"writing image in to {file_name}")
        cv2.imwrite(file_name, cv2_img)
        count += 1


def main():
    rospy.init_node('task_III_subscriber')

    # Set up your subscriber and define its callback
    # TODO 4: decide which topic should be subscribed,
    # and the related callback function name.
    # Write down your subscriber.

    image_sub = rospy.Subscriber(
        "/camera/color/image_raw", Image, callbackFunction_I)
    image_sub = rospy.Subscriber(
        "/camera/depth/image_rect_raw", Image, callbackFunction_II)

    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    # TODO 4.5: maybe you will need a counter here
    count = 1
    main()
