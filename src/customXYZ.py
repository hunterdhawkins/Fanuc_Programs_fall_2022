#! /usr/bin/env python

import controller
from fanuc_demo.msg import fullCoordinate
import rospy
#import time

# Main function.
def main():
    # Get controller for robot.
    fanuc = controller.FanucInterface()
    fanuc.setOrientationTolerance(0.1)

    print("Moving to starting position")
    joints = [-0.6, 1.2, 0, -1.57, 1.3, 1.35]
    fanuc.go_to_joint_state(joints)

    print("Moving down")
    joints = [-0.6, 1.29, 0, -1.57, 1.3, 1.35]
    fanuc.go_to_joint_state(joints)

    """
    print("Moving sideways")
    joints = [-0.4, 1.1, -0.3, -1.57, 1.3, 1.35]
    fanuc.go_to_joint_state(joints)
    """

# Boilerplate.
if __name__ == '__main__':
    main()
