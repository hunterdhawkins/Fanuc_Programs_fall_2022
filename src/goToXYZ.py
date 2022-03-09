#! /usr/bin/env python

import controller
from fanuc_demo.msg import fullCoordinate
import rospy
#import time

# Makes it easier to control the robot's XYZ position.
class PositionController:
    def __init__(self, f, x=0, y=0, z=0, angle1=0, angle2=0, angle3=0, angle4=0):
        self.fanuc = f
        self.x = x
        self.y = y
        self.z = z
        self.angle1 = angle1
        self.angle2 = angle2
        self.angle3 = angle3
        self.angle4 = angle4

    def set(self, x, y, z, angle1, angle2, angle3, angle4):
        self.x = x
        self.y = y
        self.z = z
        self.angle1 = angle1
        self.angle2 = angle2
        self.angle3 = angle3
        self.angle4 = angle4


    def get(self):
        position = [self.x, self.y, self.z, self.angle1, self.angle2, self.angle3, self.angle4]
        return position

    def move(self):
        self.fanuc.go_to_pose_goal(self.get())

# Main function.
def main():
    # Get controller for robot.
    fanuc = controller.FanucInterface()

    # Go to starting orientation.
    print("Moving to starting orientation")
    joints = [-0.75, 0, 0, 0, 0, 0]
    fanuc.go_to_joint_state(joints)

    # Create object for controlling XYZ position and move to it.
    print("Moving to right above table")
    pos = PositionController(fanuc, 0, 0.75, 1.5, 0.0870129, -0.0676836, -0.64532, 0.755912)
    pos.move()

    # Move side to side across the table 3 times.
    print("Moving back and forth")
    for i in range(0, 3):
        pos.x += 1
        pos.move()
        pos.x -= 2
        pos.move()
        pos.x += 1

# Boilerplate.
if __name__ == '__main__':
    main()
