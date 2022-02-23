#! /usr/bin/env python

import controller
from fanuc_demo.msg import fullCoordinate
import rospy
#import time

# Makes it easier to control the robot's XYZ position.
class PositionController:
    def __init__(self, f, x=0, y=0, z=0):
        self.fanuc = f
        self.x = x
        self.y = y
        self.z = z

    def set(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get(self):
        position = [self.x, self.y, self.z]
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
    pos = PositionController(fanuc, 0, 0.75, 1.2)
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
