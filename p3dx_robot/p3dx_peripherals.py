#! /usr/bin/python3
#   JTM 2022
#   port of arduino-songs library to python script with similar functionality
#   current note is sent to arduino after timer expires
#   library can be found at https://github.com/robsoncouto/arduino-songs

import time

#predefined note frequencies, taken from library
NOTE_B0 = 31
NOTE_C1 = 33
NOTE_CS1 = 35
NOTE_D1 = 37
NOTE_DS1 = 39
NOTE_E1 = 41
NOTE_F1 = 44
NOTE_FS1 = 46
NOTE_G1 = 49
NOTE_GS1 = 52
NOTE_A1 = 55
NOTE_AS1 = 58
NOTE_B1 = 62
NOTE_C2 = 65
NOTE_CS2 = 69
NOTE_D2 = 73
NOTE_DS2 = 78
NOTE_E2 = 82
NOTE_F2 = 87
NOTE_FS2 = 93
NOTE_G2 = 98
NOTE_GS2 = 104
NOTE_A2 = 110
NOTE_AS2 = 117
NOTE_B2 = 123
NOTE_C3 = 131
NOTE_CS3 = 139
NOTE_D3 = 147
NOTE_DS3 = 156
NOTE_E3 = 165
NOTE_F3 = 175
NOTE_FS3 = 185
NOTE_G3 = 196
NOTE_GS3 = 208
NOTE_A3 = 220
NOTE_AS3 = 233
NOTE_B3 = 247
NOTE_C4 = 262
NOTE_CS4 = 277
NOTE_D4 = 294
NOTE_DS4 = 311
NOTE_E4 = 330
NOTE_F4 = 349
NOTE_FS4 = 370
NOTE_G4 = 392
NOTE_GS4 = 415
NOTE_A4 = 440
NOTE_AS4 = 466
NOTE_B4 = 494
NOTE_C5 = 523
NOTE_CS5 = 554
NOTE_D5 = 587
NOTE_DS5 = 622
NOTE_E5 = 659
NOTE_F5 = 698
NOTE_FS5 = 740
NOTE_G5 = 784
NOTE_GS5 = 831
NOTE_A5 = 880
NOTE_AS5 = 932
NOTE_B5 = 988
NOTE_C6 = 1047
NOTE_CS6 = 1109
NOTE_D6 = 1175
NOTE_DS6 = 1245
NOTE_E6 = 1319
NOTE_F6 = 1397
NOTE_FS6 = 1480
NOTE_G6 = 1568
NOTE_GS6 = 1661
NOTE_A6 = 1760
NOTE_AS6 = 1865
NOTE_B6 = 1976
NOTE_C7 = 2093
NOTE_CS7 = 2217
NOTE_D7 = 2349
NOTE_DS7 = 2489
NOTE_E7 = 2637
NOTE_F7 = 2794
NOTE_FS7 = 2960
NOTE_G7 = 3136
NOTE_GS7 = 3322
NOTE_A7 = 3520
NOTE_AS7 = 3729
NOTE_B7 = 3951
NOTE_C8 = 4186
NOTE_CS8 = 4435
NOTE_D8 = 4699
NOTE_DS8 = 4978
REST = 0

#songs array to store arrays of notes (note_name, duration)
#array entries must have an even number of items, each note must have a pitch and duration
#the following songs were taken from the arduino-songs examples
songs = [		
			#song 0 is nothing
			[
			REST,1,
			],
			#song 1 is keyboard cat theme
			[
		    REST,1,
		    NOTE_C4,4, NOTE_E4,4, NOTE_G4,4, NOTE_E4,4, 
		    NOTE_C4,4, NOTE_E4,8, NOTE_G4,-4, NOTE_E4,4,
		    NOTE_A3,4, NOTE_C4,4, NOTE_E4,4, NOTE_C4,4,
		    NOTE_A3,4, NOTE_C4,8, NOTE_E4,-4, NOTE_C4,4,
		    NOTE_G3,4, NOTE_B3,4, NOTE_D4,4, NOTE_B3,4,
		    NOTE_G3,4, NOTE_B3,8, NOTE_D4,-4, NOTE_B3,4,

		    NOTE_G3,4, NOTE_G3,8, NOTE_G3,-4, NOTE_G3,8, NOTE_G3,4, 
		    NOTE_G3,4, NOTE_G3,4, NOTE_G3,8, NOTE_G3,4,
		    NOTE_C4,4, NOTE_E4,4, NOTE_G4,4, NOTE_E4,4, 
		    NOTE_C4,4, NOTE_E4,8, NOTE_G4,-4, NOTE_E4,4,
		    NOTE_A3,4, NOTE_C4,4, NOTE_E4,4, NOTE_C4,4,
		    NOTE_A3,4, NOTE_C4,8, NOTE_E4,-4, NOTE_C4,4,
		    NOTE_G3,4, NOTE_B3,4, NOTE_D4,4, NOTE_B3,4,
		    NOTE_G3,4, NOTE_B3,8, NOTE_D4,-4, NOTE_B3,4,
			NOTE_G3,-1,
			REST,1
			],
			#song 2 is mii channel theme
			[
			REST,1,
			NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, #1
			NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_CS4,8,
			NOTE_D4,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
			NOTE_E5,-4, NOTE_DS5,8, NOTE_D5,8, REST,8, REST,4,
			  
			NOTE_GS4,8, REST,8, NOTE_CS5,8, NOTE_FS4,8, REST,8,NOTE_CS5,8, REST,8, NOTE_GS4,8, #5
			REST,8, NOTE_CS5,8, NOTE_G4,8, NOTE_FS4,8, REST,8, NOTE_E4,8, REST,8,
			NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,8, REST,4, NOTE_E4,8, NOTE_E4,8,
			NOTE_E4,8, REST,8, REST,4, NOTE_DS4,8, NOTE_D4,8, 

			NOTE_CS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, #9
			NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, NOTE_E5,8, NOTE_E5,8, NOTE_E5,8, REST,8,
			REST,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
			NOTE_E5,2, NOTE_D5,8, REST,8, REST,4,

			NOTE_B4,8, NOTE_G4,8, NOTE_D4,8, NOTE_CS4,4, NOTE_B4,8, NOTE_G4,8, NOTE_CS4,8, #13
			NOTE_A4,8, NOTE_FS4,8, NOTE_C4,8, NOTE_B3,4, NOTE_F4,8, NOTE_D4,8, NOTE_B3,8,
			NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,4, REST,4, NOTE_AS4,4,
			NOTE_CS5,8, NOTE_D5,8, NOTE_FS5,8, NOTE_A5,8, REST,8, REST,4, 

			REST,2, NOTE_A3,4, NOTE_AS3,4, #17 
			NOTE_A3,-4, NOTE_A3,8, NOTE_A3,2,
			REST,4, NOTE_A3,8, NOTE_AS3,8, NOTE_A3,8, NOTE_F4,4, NOTE_C4,8,
			NOTE_A3,-4, NOTE_A3,8, NOTE_A3,2,

			REST,2, NOTE_B3,4, NOTE_C4,4, #21
			NOTE_CS4,-4, NOTE_C4,8, NOTE_CS4,2,
			REST,4, NOTE_CS4,8, NOTE_C4,8, NOTE_CS4,8, NOTE_GS4,4, NOTE_DS4,8,
			NOTE_CS4,-4, NOTE_DS4,8, NOTE_B3,1,
			  
			NOTE_E4,4, NOTE_E4,4, NOTE_E4,4, REST,8,#25

			#repeats 1-25

			NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, #1
			NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_CS4,8,
			NOTE_D4,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
			NOTE_E5,-4, NOTE_DS5,8, NOTE_D5,8, REST,8, REST,4,
			  
			NOTE_GS4,8, REST,8, NOTE_CS5,8, NOTE_FS4,8, REST,8,NOTE_CS5,8, REST,8, NOTE_GS4,8, #5
			REST,8, NOTE_CS5,8, NOTE_G4,8, NOTE_FS4,8, REST,8, NOTE_E4,8, REST,8,
			NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,8, REST,4, NOTE_E4,8, NOTE_E4,8,
			NOTE_E4,8, REST,8, REST,4, NOTE_DS4,8, NOTE_D4,8, 

			NOTE_CS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, #9
			NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, NOTE_E5,8, NOTE_E5,8, NOTE_E5,8, REST,8,
			REST,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
			NOTE_E5,2, NOTE_D5,8, REST,8, REST,4,

			NOTE_B4,8, NOTE_G4,8, NOTE_D4,8, NOTE_CS4,4, NOTE_B4,8, NOTE_G4,8, NOTE_CS4,8, #13
			NOTE_A4,8, NOTE_FS4,8, NOTE_C4,8, NOTE_B3,4, NOTE_F4,8, NOTE_D4,8, NOTE_B3,8,
			NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,4, REST,4, NOTE_AS4,4,
			NOTE_CS5,8, NOTE_D5,8, NOTE_FS5,8, NOTE_A5,8, REST,8, REST,4, 

			REST,2, NOTE_A3,4, NOTE_AS3,4, #17 
			NOTE_A3,-4, NOTE_A3,8, NOTE_A3,2,
			REST,4, NOTE_A3,8, NOTE_AS3,8, NOTE_A3,8, NOTE_F4,4, NOTE_C4,8,
			NOTE_A3,-4, NOTE_A3,8, NOTE_A3,2,

			REST,2, NOTE_B3,4, NOTE_C4,4, #21
			NOTE_CS4,-4, NOTE_C4,8, NOTE_CS4,2,
			REST,4, NOTE_CS4,8, NOTE_C4,8, NOTE_CS4,8, NOTE_GS4,4, NOTE_DS4,8,
			NOTE_CS4,-4, NOTE_DS4,8, NOTE_B3,1,
			  
			NOTE_E4,4, NOTE_E4,4, NOTE_E4,4, REST,8,#25

			#finishes with 26
			NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_FS4,8,
			REST,1
			],
			#
			[
			REST,1,
			NOTE_E5, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_C5,8,  NOTE_B4,8,
			NOTE_A4, 4,  NOTE_A4,8,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
			NOTE_B4, -4,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
			NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,8,  NOTE_A4,4,  NOTE_B4,8,  NOTE_C5,8,

			NOTE_D5, -4,  NOTE_F5,8,  NOTE_A5,4,  NOTE_G5,8,  NOTE_F5,8,
			NOTE_E5, -4,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
			NOTE_B4, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
			NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,4, REST, 4,

			NOTE_E5, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_C5,8,  NOTE_B4,8,
			NOTE_A4, 4,  NOTE_A4,8,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
			NOTE_B4, -4,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
			NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,8,  NOTE_A4,4,  NOTE_B4,8,  NOTE_C5,8,

			NOTE_D5, -4,  NOTE_F5,8,  NOTE_A5,4,  NOTE_G5,8,  NOTE_F5,8,
			NOTE_E5, -4,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
			NOTE_B4, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
			NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,4, REST, 4,
			  

			NOTE_E5,2,  NOTE_C5,2,
			NOTE_D5,2,   NOTE_B4,2,
			NOTE_C5,2,   NOTE_A4,2,
			NOTE_GS4,2,  NOTE_B4,4,  REST,8, 
			NOTE_E5,2,   NOTE_C5,2,
			NOTE_D5,2,   NOTE_B4,2,
			NOTE_C5,4,   NOTE_E5,4,  NOTE_A5,2,
			NOTE_GS5,2,
			REST,1
			]
		]

#music box class acts like a music box
#start the music box by creating a music_box object and call get_note to get the current pitch
#all calculations and timekeeping is done within the class
#use this by calling the object, calling get_note and monitoring the output freq
#when the output freq changes, output the new freq to buzzer 
class music_box:
	#arguments are: song array index (or custom array with same format) and a song tempo
	def __init__(self, song, tempo=160):
		self.tempo = 160.0
		self.song = song
		self.notes = (len(self.song)/2)-1
		self.current_note = 0
		self.wholenote = (60 * 4) / self.tempo
		self.divider = self.song[(self.current_note*2)+1]
		self.noteduration = self.wholenote/self.divider
		self.last_note_change = time.perf_counter()
		self.note_on = True

	#function to move to the next note in the song
	#will calculate dependant variables and handle a dotted note (a negative duration)
	def advance_note(self):
		self.current_note += 1
		self.divider = self.song[(self.current_note*2)+1]
		self.noteduration = self.wholenote/self.divider
		if self.noteduration > 0:
			self.noteduration = self.wholenote/self.divider
		if self.noteduration < 0:
			self.noteduration = self.wholenote/self.divider
			self.noteduration *= 1.5

	#function to get the frequency of the current note
	#notes are played for 90% duration before a freq of 0 is send for the last 10%
	#this makes the notes more distinguished
	#frequency of current note is returned when called, if song is finished then None is returned
	def get_note(self):
		#if the current note is not the last note
		if self.current_note < self.notes:
			#get current time
			curr_time = time.perf_counter()
			#check if the note is playing and has reached 90% duration
			#record current time as last not change, set note_on to false
			if (self.last_note_change+(self.noteduration*0.9)) < curr_time and self.note_on == True:
				self.last_note_change = curr_time
				self.note_on = False
			#check if the note is not playing and has reached 10% duration
			#record current time as last not change, set note_on to true and advance to next note
			if (self.last_note_change+(self.noteduration*0.1)) < curr_time and self.note_on == False:
				self.last_note_change = curr_time
				self.note_on = True
				self.advance_note()
			#if the note is on, assign the note frequency to freq
			#otherwise set freq to 0
			if self.note_on:
				freq = self.song[(self.current_note*2)]
			else:
				freq = 0

			#return the set freq
			return freq

		return None

