#! /usr/bin/env python

import controller
import rospy
import time
from fanuc_demo.msg import fullCoordinate
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16

note_array1 = [18, 18, 18, 18, 18, 18,
               17, 17, 17, 17, 17, 17,
               16, 16, 16, 16, 16, 16,
               15, 15, 15, 15, 16, 17, 
               18, 18, 18, 18, 18, 18,
               17, 17, 17, 17, 17, 17,
               16, 16, 16, 16, 16, 16, 
               15, 15, 18, 15]

note_array2 = [19, 19, 19, 19, 19, 19,
               19, 19, 19, 19, 19, 19,
               21, 21, 21, 21, 20, 21,
               22, 22, 22, 22, 21, 20, 
               19, 19, 19, 19, 19, 19,
               19, 19, 19, 19, 19, 19,
               21, 21, 21, 21, 20, 21,
               22, 22, 19, 22]



# Makes it easier to control the robot's XYZ position.
class PositionController:
    def __init__(self, f, x=0, y=0, z=0, angle1=0.0870129, angle2=-0.0676836, angle3=-0.6453200, angle4=0.7559120):
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
        position = [self.x, self.y, self.z, self.angle1, self.angle2, self.angle3, self.angle4]
        self.fanuc.go_to_pose_goal(position)



# Each state is a function
# the function should handle dealing with the situation that the state represents
# and switch to other states at predetermined spots based upon conditions
class RoboticArm():
    def __init__(self, pub):
        self.fanuc = controller.FanucInterface() #get controller for robot
        self.pub = pub
        self.claw_spacing = 0
        self.note_index = 0
        self.states = {
            "move_to_ready_state": self.move_to_ready_state,
            "move_to_key_position": self.move_to_key_position,
            "play_note": self.play_note,
            "end_of_song": self.end_of_song}
        self.state = "move_to_ready_state"
        self.run_state()


    def run_state(self):
        self.states[self.state]()


    # In this state we will move from the robotic arms resting state to our ready state
    def move_to_ready_state(self):
        print("Moving to the ready state which is the middle of the piano")

        # Lift arm up to avoid hitting piano during move.
        init_joint_goal = self.fanuc.move_group.get_current_joint_values() 
        init_joint_goal[1] = 1.0
        self.fanuc.go_to_joint_state(init_joint_goal)

        # Set position to the middle.
        note = 14.0
        self.pos = PositionController(
            self.fanuc,
            -0.771 + (note * 0.0228),
            1.29,
            0.95 + (note * 0.00143))

        # Move arm.
        self.pos.move()

        # Go to next state.
        print("We have reached the ready state, now ready to play")
        self.state = "move_to_key_position"
        self.run_state()


    # This state moves the claw to the correct XYZ position to play the next note(s).
    def move_to_key_position(self):
        print("********************Moving to key position **************************")

        # Stop if all of the notes have been played.
        if self.note_index == len(note_array1):
            self.state = "end_of_song"
            self.run_state()

        # Get the next notes.
        note1 = note_array1[self.note_index]
        note2 = note_array2[self.note_index]

        # Find the position and claw spacing, and send the spacing to the claw.
        claw_spacing = abs(note1 - note2)
        self.pub.publish(claw_spacing)
        note = (note1 + note2)/2.0
        print(note)

        # Set new position.
        self.pos.set(
            -0.771 + (note * 0.0228),
            1.29,
            0.95 + (note * 0.00143) + (claw_spacing * -0.0015))

        # Orient claw.
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

        # Move arm.
        self.pos.move()

        # Go to the next state.
        self.state = "play_note"
        self.run_state()


    # This state will play a note for a certain amount of time and then move to the "move to key position" state.
    def play_note(self):
        print("********************Playing Note**************************")

        # Move down and up.
        self.pos.z -= 0.05
        self.pos.move()
        self.pos.z += 0.05
        self.pos.move()

        # Continue to next note(s).
        self.note_index += 1

        # Go to the next state.
        self.state = "move_to_key_position"
        self.run_state()


    # When the song ends, this state stops the program.
    def end_of_song(self):
        print("Done with song")
        quit()	



# Main function.
def main():
    pub = rospy.Publisher('chatter', Int16, queue_size=10)
    arm = RoboticArm(pub)
    arm.run_state()



# Boilerplate.
if __name__ == '__main__':
    main()
