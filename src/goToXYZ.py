#! /usr/bin/env python

import controller
from fanuc_demo.msg import fullCoordinate
import rospy
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
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

    def set(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get(self):
        position = [self.x, self.y, self.z, self.angle1, self.angle2, self.angle3, self.angle4]
        return position

    def move(self):
        temp_pos = [self.x, self.y, self.z, self.angle1, self.angle2, self.angle3, self.angle4]
        #if self.x > 0:
        #    for i in range(3, 7):
        #        temp_pos[i] *= -1
        #print(temp_pos)
        self.fanuc.go_to_pose_goal(temp_pos)

        print("")
        print(self.fanuc.move_group.get_current_pose().pose)

# Main function.
def main():
    # Get controller for robot.
    fanuc = controller.FanucInterface()

    """
    # Go to starting orientation.
    print("Moving to starting orientation")
    joints = [-0.75, 0, 0, 0, 0, 0]
    fanuc.go_to_joint_state(joints)
    """

    # Create object for controlling XYZ position and move to it.
    print("Moving to right above table")

    # X diff = 0.63, Z diff = 0.04
    # Note X diff = 0.0225, Note Z diff = 0.00143
    # 0 = leftmost note, 28 = rightmost note

    note = 28
    pos = PositionController(fanuc, -0.09 + (note * -0.0225), 1.3, 1.0 + (note * -0.00143), 0.0870129, -0.0676836, -0.64532, 0.755912)

    ori_list = [pos.angle1, pos.angle2, pos.angle3, pos.angle4]
    rpy_tuple = euler_from_quaternion(ori_list)
    
    rpy_list = [0.0, 0.0, 0.0]
    rpy_list[0] = rpy_tuple[0]
    rpy_list[1] = rpy_tuple[1] - 0.5
    rpy_list[2] = rpy_tuple[2]

    ori_list = quaternion_from_euler(rpy_list[0], rpy_list[1], rpy_list[2])
    pos.angle1 = ori_list[0]
    pos.angle2 = ori_list[1]
    pos.angle3 = ori_list[2]
    pos.angle4 = ori_list[3]

    pos.move()
    pos.z -= 0.05
    pos.move()
    pos.z += 0.05
    pos.move()

# Boilerplate.
if __name__ == '__main__':
    main()
