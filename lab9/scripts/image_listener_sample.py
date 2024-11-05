#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


bridge = CvBridge()


def callback(image_message):

    # TODO: implement a callback funtion to show the image_message
    # Note: a image can be shown with opencv (cv2.imshow() function)
    # Note: remember wait for 10ms using waitKey() function after calling cv2.imshow()
    # Note: refer to lab7 taskII.py
    # Line1 using bridge package
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message, 'bgr8')

        cv2.imshow("Image Window", cv_image)

        cv2.waitKey(10)
    except Exception as e:
        print(e)


def main():
    rospy.init_node("color", anonymous=True)

    # TODO: define a subscriber that subscribe 'Image' message from appropriate topic
    # Line1 set up subscriber and check the topic by yourself

    ##
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.spin()


if __name__ == '__main__':

    main()
