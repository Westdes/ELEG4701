#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from lab5.srv import *


def multiply_two_floats_client(x, y):
    rospy.wait_for_service('multiply_two_floats')
    try:
        multiply_two_floats = rospy.ServiceProxy(
            'multiply_two_floats', MultiplyTwoFloats)
        resp1 = multiply_two_floats(x, y)
        return resp1.Product
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "%s [x y]" % sys.argv[0]


if __name__ == "__main__":
    rospy.init_node('multiply_two_floats_client', anonymous=True)

    if rospy.has_param('~x') and rospy.has_param('~y'):
        x = rospy.get_param('~x')
        y = rospy.get_param('~y')
    else:
        print(usage())
        sys.exit(1)

    print("Requesting %s*%s" % (x, y))
    print("%s * %s = %s" % (x, y, multiply_two_floats_client(x, y)))
