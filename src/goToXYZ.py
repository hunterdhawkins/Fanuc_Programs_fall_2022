#! /usr/bin/env python

#import numpy as np
import controller
from fanuc_demo.msg import fullCoordinate
import rospy
#import math
#import time

def main():
    # Get controller for robot.
    fanuc = controller.FanucInterface()

    # Go to starting position.
    #fanuc.zeroOut()
    joints = [-0.75, 0, 0, 0, 0, 0]
    fanuc.go_to_joint_state(joints)

    # Move to XYZ position.
    points = [0, 0.75, 1.2]
    fanuc.go_to_pose_goal(points)

    #print(fanuc.move_group.get_current_joint_values())
    #print(fanuc.move_group.get_current_pose().pose.position)

if __name__ == '__main__':
    main()
