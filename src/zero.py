#! /usr/bin/env python
import numpy as np
import controller 
from fanuc_demo.msg import fullCoordinate
import rospy
import math

#This program simply returns the robot to it's zero point in regard
#to joint coordinates


def main():
   fanuc = controller.FanucInterface()
   fanuc.zeroOut()


if __name__=="__main__":
    main()
