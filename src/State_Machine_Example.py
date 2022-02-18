import time


'''####################################################################################
									Notes
# Each key appears to be 14/16th of an inch (0.875 inches)
# 49 keys on piano



####################################################################################'''


# Measure the distance between keys to see what we can reach


# Each state is a function
# the function should handle dealing with the situation that the state represents
# and switch to other states at predetermined spots based upon conditions
	
class RoboticArm():
	def __init__(self):


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
		lined_up = True
		if lined_up:
			print("We have reached the ready state, now ready to play")
			self.state = "move_to_key_position"
			time.sleep(2)
			self.run_state()


	#In our case here the lined_up flag would be triggered by knowing the X,Y,Z needed to be lined up with the key
	#We would do this by Comparing our current position with that known X,Y,Z, and if we are within a certain threshould (2mm) then move to the play note state
	def move_to_key_position(self):
		print("Moving to the position of the key")
		lined_up = True
		if lined_up:
			print("We are in position")
			self.state = "play_note"
			time.sleep(2)
			self.run_state()


	#This state will play a note for a certain amount of time and than move to the move to key psoition state	
	def play_note(self):
		print("********************Playing Note**************************")
		self.state = "move_to_key_position"
		time.sleep(2)
		self.run_state()
	

	
	def run_state(self):
		self.states[self.state]()





def main():
	arm=RoboticArm()
	arm.run_state()

main()