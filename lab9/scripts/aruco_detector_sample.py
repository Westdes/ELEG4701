#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import math
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def callback(ros_pics):
    my_image = bridge.imgmsg_to_cv2(ros_pics)
    my_image = cv2.cvtColor(my_image, cv2.COLOR_BGR2RGB)
    current_frame_gray = cv2.cvtColor(my_image, cv2.COLOR_BGR2GRAY)
    # detect aruco marker
    corners, aruco_ids, _ = cv2.aruco.detectMarkers(
        current_frame_gray, aruco_dictionary, parameters=aruco_parameters)
    # draw the corners and edges of the marker on the image
    my_image = cv2.aruco.drawDetectedMarkers(
        image=my_image, corners=corners, ids=aruco_ids, borderColor=(0, 255, 0))

    # TODO: show my_image with opencv functions, like what you have done in Task II (window name: 'aruco id')
    cv2.imshow("Image Window", my_image)
    cv2.waitKey(10)

    ##

    if aruco_ids is not None:
        # rvecs and tvecs are the rotation and translation vectors respectively, for each of the markers in corners.
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, 1, intrinsic_camera_matrix, intrinsic_camera_distortion)
        # print(rvecs.shape)
        for rvec, tvec in zip(rvecs, tvecs):
            # draw axis on the Aruco marker
            cv2.aruco.drawAxis(my_image, intrinsic_camera_matrix,
                               intrinsic_camera_distortion, rvec, tvec, 1)
            # broadcast the transformation of the marker
            transform.header.stamp = rospy.Time.now()
            transform.transform.translation.x = -tvec[0, 1] / 10.0
            transform.transform.translation.y = tvec[0, 0]/10.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            br.sendTransform(transform)

        # TODO: show my_image with opencv functions, like what you have done in Task II (window name: 'axis')
        cv2.imshow("axis", my_image)
        cv2.waitKey(10)

        ##

    else:
        # if no markers are detected, broadcast tranformations as zeros.
        print('No marker')
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        br.sendTransform(transform)


if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster')

    # TODO: Fill in correct rostopic
    sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    ##

    br = tf2_ros.TransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()

    transform.header.frame_id = "base_link"
    transform.child_frame_id = "aruco"
    bridge = CvBridge()

    # Specify the dictionary of aurco marker
    aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    # Create an parameter structure needed for the ArUco detection
    aruco_parameters = cv2.aruco.DetectorParameters_create()
    # Specify the parameter for: corner refinement
    aruco_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    # Specify the intrinsic parameters of the camera

    # TODO: substitute the '0.0' with the results of your calibration in Task II
    intrinsic_camera_matrix = np.array(
        [[687.955288, 0.0, 342.967920], [0.0, 681.452386, 252.084572], [0.0, 0.0, 1.0]], dtype=float)
    intrinsic_camera_distortion = np.array(
        [[0.234943, -0.351335, 0.001871, 0.024800, 0.0]], dtype=float)
    ##

    rospy.spin()
