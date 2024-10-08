#!/usr/bin/env python

from __future__ import print_function

from lab5.srv import *
import rospy


def handle_multiply_two_floats(req):
    print("Returning [%s * %s = %s]" % (req.A, req.B, (req.A * req.B)))
    return MultiplyTwoFloatsResponse(req.A * req.B)


def multiply_two_floats_server():
    rospy.init_node('multiply_two_floats_server')
    s = rospy.Service('multiply_two_floats', MultiplyTwoFloats,
                      handle_multiply_two_floats)
    print("Ready to multiply two floats.")
    rospy.spin()


if __name__ == "__main__":
    multiply_two_floats_server()
