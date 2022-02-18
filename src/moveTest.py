#! /usr/bin/env python

import numpy as np
import controller
from fanuc_demo.msg import fullCoordinate
import rospy
import math
import time

def main():
    fanuc = controller.FanucInterface()
    time.sleep(5)
    joints = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

    for i in range(1, 4):
        fanuc.go_to_joint_state(joints)
        time.sleep(5)
        fanuc.zeroOut()
        time.sleep(5)

if __name__ == '__main__':
    main()
