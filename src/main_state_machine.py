#! /usr/bin/env python

import controller
from fanuc_demo.msg import fullCoordinate
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import rospy
import time
from moveit_msgs.msg import Constraints


from std_msgs.msg import String, Int16

'''####################################################################################
									Notes
# Each key appears to be 14/16th of an inch (0.875 inches)
# 49 keys on piano



####################################################################################'''


# Measure the distance between keys to see what we can reach


# Each state is a function
# the function should handle dealing with the situation that the state represents
# and switch to other states at predetermined spots based upon conditions
	

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




class RoboticArm():
    def __init__(self, pub):
        constraints = Constraints()
        print(constraints)

        # Get controller for robot.
        self.fanuc = controller.FanucInterface()
        self.pub = pub
        self.claw_spacing = 0
        self.state = "move_to_ready_state"
        self.states = {
			"move_to_ready_state": self.move_to_ready_state,
			"move_to_key_position": self.move_to_key_position,
			"play_note": self.play_note
		}
        self.run_state()


	#In this state we will move from the robotic arms resting state to our ready state
	#Ideally this will be to the left or right side of the piano
    def move_to_ready_state(self):
        print("Moving to the ready state which is the right or left side of the piano")

        init_joint_goal = self.fanuc.move_group.get_current_joint_values() 
        init_joint_goal[1] = 1.0
        self.fanuc.go_to_joint_state(init_joint_goal)
        
        # X diff = 0.63, Z diff = 0.04
        # Note X diff = 0.0225, Note Z diff = 0.00143
        # 0 = leftmost note, 28 = rightmost note


        #need to account for how wide open the claw is and adjust z accordingly

        # Choose starting note.
        #these will have values of 0-28
        self.note1 = 0.0
        self.note2 = 7.0

        note = (self.note1 + self.note2)/2.0

        self.pos = PositionController(
            self.fanuc
            ,-0.771 + (note * 0.0228)
            ,1.29
            ,0.95  + (note * 0.00143)
            ,0.0870129
            ,-0.0676836
            ,-0.64532
            ,0.755912)


        # Tilt claw down.
        ori_list = [self.pos.angle1, self.pos.angle2, self.pos.angle3, self.pos.angle4]
        rpy_tuple = euler_from_quaternion(ori_list)
        rpy_list = [0.0, 0.0, 0.0]
        rpy_list[0] = rpy_tuple[0] - 0.1
        rpy_list[1] = rpy_tuple[1] - 0.5
        rpy_list[2] = rpy_tuple[2] - 0.2
        ori_list = quaternion_from_euler(rpy_list[0], rpy_list[1], rpy_list[2])
        self.pos.angle1 = ori_list[0]
        self.pos.angle2 = ori_list[1]
        self.pos.angle3 = ori_list[2]
        self.pos.angle4 = ori_list[3]


        self.pos.move()

        lined_up = True
        if lined_up:
        	print("We have reached the ready state, now ready to play")
        	self.state = "move_to_key_position"
        	self.run_state()


	#In our case here the lined_up flag would be triggered by knowing the X,Y,Z needed to be lined up with the key
	#We would do this by Comparing our current position with that known X,Y,Z, and if we are within a certain threshould (2mm) then move to the play note state
    def move_to_key_position(self):

        print("********************Moving to key position **************************")

        self.note1 += 1.0
        self.note2 += 1.0


        claw_spacing = abs(self.note1 - self.note2)
        self.pub.publish(claw_spacing)

        note = (self.note1 + self.note2)/2.0

        print(note)


        self.pos = PositionController(self.fanuc,
        -0.771 + (note * 0.0228)
        , 1.29
        , 0.95  + (note * 0.00143) + (claw_spacing * -0.0015) # + something for how wide claw is      
        , 0.0870129
        , -0.0676836
        , -0.64532
        , 0.755912)

        # Tilt claw down.
        ori_list = [self.pos.angle1, self.pos.angle2, self.pos.angle3, self.pos.angle4]
        rpy_tuple = euler_from_quaternion(ori_list)
        rpy_list = [0.0, 0.0, 0.0]
        rpy_list[0] = rpy_tuple[0] - 0.1
        rpy_list[1] = rpy_tuple[1] - 0.5
        rpy_list[2] = rpy_tuple[2] - 0.2
        ori_list = quaternion_from_euler(rpy_list[0], rpy_list[1], rpy_list[2])
        self.pos.angle1 = ori_list[0]
        self.pos.angle2 = ori_list[1]
        self.pos.angle3 = ori_list[2]
        self.pos.angle4 = ori_list[3]



        self.pos.move()
        self.state = "play_note"
        self.run_state()


	#This state will play a note for a certain amount of time and than move to the move to key psoition state	
    def play_note(self):
        print("********************Playing Note**************************")

        self.pos.z -= 0.05
        self.pos.move()
        self.pos.z += 0.05
        self.pos.move()

        
        self.state = "move_to_key_position"
        self.run_state()
        
	

    def run_state(self):
        self.states[self.state]()




def main():
    pub = rospy.Publisher('chatter', Int16 , queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz


    arm=RoboticArm(pub)
    arm.run_state()

main()
