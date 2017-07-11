#!/usr/bin/env python
from rosparam_handler_tutorial.demo_python import Demo
import sys
import rospy

def main(args):
    """
    Initializes and cleanup ros node
    """
    rospy.init_node('demo_node')
    demo_node = Demo()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
