#! /usr/bin/env python
import numpy as np
import controller 
from fanuc_demo.msg import fullCoordinate
import rospy
import math

#This program simply returns the robot to it's zero point in regard
#to joint coordinates


def main():
   #Instantiate and setup scene
   fanuc = controller.FanucInterface()
   fanuc.zeroOut()
   fanuc.setupScene()
   # point1 = [0.3, 0.4, 2.0]
   # point2 = [0.4, 0.1, 0.6, 1.2]
   fanuc.setOrientationTolerance(0.1)
   # fanuc.orientationAnyPosition(point2)
   fanuc.setPositionTolerance(0.1)
   # fanuc.positionAnyOrientation(point1)
   while(1):
      fanuc.randomJointMove()



if __name__=="__main__":
    main()
