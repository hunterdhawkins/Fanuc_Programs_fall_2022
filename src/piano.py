#! /usr/bin/env python
import controller
import rospy
import time
import os
from fanuc_demo.msg import fullCoordinate
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16

# Text representation of each note.
note_character = ['XX', 'C2', 'D2', 'E2', 'F2', 'G2', 'A2', 'B2',
                        'C3', 'D3', 'E3', 'F3', 'G3', 'A3', 'B3',
                        'C4', 'D4', 'E4', 'F4', 'G4', 'A4', 'B4',
                        'C5', 'D5', 'E5', 'F5', 'G5', 'A5', 'B5',
                        'C6']



# Reads a line from a song file into an array.
def read_in_from_file(arm, array_num):
    # Read until a newline is reached.
    note_number = ""
    next_char = "start"
    while not (next_char == "\n" or next_char == ""):
        # Get the next character from the file.
        next_char = arm.filename.read(1);

        # Check what character was read.
        if next_char == "," or next_char == "\n" or next_char == "":
            # Add the note number to the array.
            if (array_num == 1):
                # Find key position from note number, and add to first array.
                key_position = note_character.index(note_number)
                arm.note_array1.append(key_position)
            elif (array_num == 2):
                # Find key position from note number, and add to second array.
                key_position = note_character.index(note_number)
                arm.note_array2.append(key_position)
            else:
                # Add the note number directly to the length array as an integer.
                arm.length_array.append(int(note_number));

            # Reset the note number to be ready to start reading the next note.
            note_number = ""
        elif not next_char.isspace():
            # Ignore whitespace (besides newlines).
            # Append the character to the note number.
            note_number += next_char



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



# Each state is a function. The function should handle dealing with the situation that the state represents, and switch to
# other states at predetermined spots based upon conditions.
class RoboticArm():
    # Initialization.
    def __init__(self, pub):
        self.fanuc = controller.FanucInterface() #get controller for robot
        self.pub = pub
        self.claw_spacing = 0
        self.note_index = 0
        self.note_array1 = []
        self.note_array2 = []
        self.length_array = []
        self.user_file_name = ""
        self.filename = ""

        #state machine logic
        self.states = {
            "move_to_ready_state": self.move_to_ready_state,
            "move_to_key_position": self.move_to_key_position,
            "play_note": self.play_note,
            "end_of_song": self.end_of_song}
        self.state = "move_to_ready_state"
        print("Moving to the ready state")
        self.run_state()


    # This function repeatedly runs states in the state machine until the state becomes "exit".
    def run_state(self):
        while self.state != "exit":
            self.states[self.state]()


    # In this state we will move from the robotic arms resting state to our ready state
    def move_to_ready_state(self):
        # Get user input, and end if "quit" is entered.
        self.user_file_name = raw_input("Enter the song you would like to play, or \"quit\" to exit, or \"list\" to list options: ")
        if self.user_file_name == "quit":
            self.state = "exit"
            return
        elif self.user_file_name == "list":
            song_list = os.listdir("songs")
            for song in song_list:
                split = song.split("_notes.txt")
                print(" - " + split[0])
            self.state = "move_to_ready_state"
            return

        # Open the file and store it into the arrays.
        self.filename = open("songs/" + self.user_file_name + "_notes.txt", "r")
        read_in_from_file(self, 1)
        read_in_from_file(self, 2)
        read_in_from_file(self, 3)
        self.filename.close()

        # Start playing.
        print("Playing song")

        # Lift arm up to avoid hitting piano during move.
        init_joint_goal = self.fanuc.move_group.get_current_joint_values() 
        init_joint_goal[1] = 1.0
        self.fanuc.go_to_joint_state(init_joint_goal)

        # Set position to the middle.
        note = 14.0
        self.pos = PositionController(
            self.fanuc,
            -0.771 + (note * 0.0228),
            1.28,
            0.96 + (note * 0.001),
            0.0870129,
            -0.0676836,
            -0.64532,
            0.755912)

        # Move arm.
        self.pos.move()

        # Go to next state.
        #print("We have reached the ready state, now ready to play")
        self.state = "move_to_key_position"


    # This state moves the claw to the correct XYZ position to play the next note(s).
    def move_to_key_position(self):
        #print("********************Moving to key position **************************")

        # Stop if all of the notes have been played.
        if self.note_index == len(self.note_array1):
            self.state = "end_of_song"
            return

        # Get the next notes.
        note1 = self.note_array1[self.note_index]
        note2 = self.note_array2[self.note_index]

        # Find the position and claw spacing, and send the spacing to the claw.        
        claw_spacing = abs(note1 - note2)
        self.pub.publish(claw_spacing)
        note = (note1 + note2)/2.0
        #print(note)

        # Move to the key's position. 
        self.pos = PositionController(
            self.fanuc,
            -0.771 + (note * 0.0228),
            1.28,
            0.96 + (note * 0.001) + (claw_spacing * -0.0017),
            0.0870129,
            -0.0676836,
            -0.64532,
            0.755912)

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


    # This state will play a note for a certain amount of time and then move to the "move to key position" state.
    def play_note(self):
        #print("********************Playing Note**************************")

        # Move down and up.
        self.pos.z -= 0.05
        self.pos.move()
        time.sleep(self.length_array[self.note_index])
        self.pos.z += 0.05
        self.pos.move()

        # Continue to next note(s).
        self.note_index += 1

        # Go to the next state.
        self.state = "move_to_key_position"


    # When the song ends, this state allows the user to select another one to play.
    def end_of_song(self):
        print("Done with song\n")
        self.state = "move_to_ready_state"



# Main function.
def main():
    pub = rospy.Publisher('chatter', Int16, queue_size=10)
    arm = RoboticArm(pub)
    arm.run_state()



# Boilerplate.
if __name__ == '__main__':
    main()

