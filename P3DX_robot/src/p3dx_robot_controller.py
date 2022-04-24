#! /usr/bin/python3
#   JTM 2022
#   the main function to run the P3-DX robot in a teleoperated mode

from email.policy import default
import time, json
from math import pi, sqrt

from _thread import *
import threading

import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32 

#function to constrain a number
def constrain(n, minn, maxn):
	return max(min(maxn, n), minn)

#class that acts as a timer
#timer object should be created, then the timer can be started with start func
#check_timer will return False until timer has expired, where it will return True
class Timer:
	#takes an interval in seconds
	def __init__(self, interval):
		self.interval = interval
		self.start_time = 0

	def start(self):
		self.start_time = time.perf_counter()

	def check_timer(self):
		if (self.start_time+self.interval) <= time.perf_counter():
			return True
		else:
			return False

# class to store P3-DX robot instance
# stores many of the variables and functions that facilitate communication between
# RPI and arduino.
class P3_DX_robot:
	# takes no arguments other than serial_port object
	def __init__(self):

		super().__init__()

		# variables to store left, right speeds and encoder values
		self.cmd_throttle = 0
		self.cmd_rotation = 0
		self.left_motor_speed = 0
		self.right_motor_speed = 0
		self.last_left_enc = 0
		self.last_right_enc = 0
		# variables to store robot button states
		self.last_reset_button_state = 1
		self.last_motor_button_state = 1
		self.last_aux1_switch_state = 1
		self.last_aux2_switch_state = 1
		# variables holding robot specific information, used for local calculations
		self.wheel_distance = 13.0
		self.wheel_diam = 7.65
		self.enc_ticks_per_rev = 19105.0
		self.wheel_dist_circum = pi*self.wheel_distance
		self.ticks_per_inch = self.enc_ticks_per_rev/(self.wheel_diam*pi)
		# robot PID constants
		self.kp = 0.5
		self.ki = 0.0
		self.kd = 0.0
		# music box object/variable for fun
		self.robot_music_box = None
		# 
		self.steering_mixgain = 0.5

		self.motor_update_timer = Timer(1.0/rospy.get_param("/robot/freqs/update_motor", 10))
		self.encoder_update_timer = Timer(1.0/rospy.get_param("/robot/freqs/update_encoder", 25))
		self.button_update_timer = Timer(1.0/rospy.get_param("/robot/freqs/update_button", 10))

		# create timer objects that generate messages to send to arduino
		self.serial_publisher = rospy.Publisher('serialcomm_message_out', String, queue_size=10)
		self.serial_publisher_timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("/serialcomm/pub_freq", 50)), self.callback_serial_publisher)
		self.serial_subscriber = rospy.Subscriber('serialcomm_message_in', String, self.callback_serial_subscriber, queue_size=10)

		self.cmd_throttle_subscriber = rospy.Subscriber('cmd_throttle', Float32, self.callback_throttle_subscriber, queue_size=1)
		self.cmd_rotation_subscriber = rospy.Subscriber('cmd_rotation', Float32, self.callback_rotation_subscriber, queue_size=1)
		self.motor_mixer_timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("/robot/freqs/motor_mixer", 25)), self.callback_steering_mixer)

		self.robot_reset_encoders_subscriber = rospy.Subscriber('robot_reset_encoders', String, self.reset_encoder_values, queue_size=1)
		self.robot_update_indicator_LED_subscriber = rospy.Subscriber('robot_update_indicator_LED', String, self.send_indicator_LED_values, queue_size=1)
		self.robot_update_PID_subscriber = rospy.Subscriber('robot_update_PID', String, self.send_pid_values, queue_size=1)
		rospy.Service("robot_estop", Trigger, self.estop)

		self.handle_buttons_timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("/robot/buttons/timer_freq", 25)), self.handle_buttons)

	def callback_throttle_subscriber(self, d):
		self.cmd_throttle = d.data

	def callback_rotation_subscriber(self, d):
		self.cmd_rotation = d.data

	# function starts all the update timers, should be done after creating this object
	def start_serial_pub_timers(self):
		self.motor_update_timer.start()
		self.encoder_update_timer.start()
		self.button_update_timer.start()

	# function to handle button states
	# will run in main loop, constantly being checked
	def handle_buttons(self, timerevent):
		# if self.last_reset_button_state == 0:
			
		# if the motorbutton state is pressed, reset motor speeds and stop script
		# this is a runaway estop method in case something goes wrong
		if self.last_motor_button_state == 0:
			self.cmd_throttle = 0
			self.cmd_rotation = 0
			self.left_motor_speed = 0
			self.right_motor_speed = 0
			rospy.loginfo("Motor button pressed, stopping motors")
		# if self.last_aux1_button_state == 0:
			
		# if self.last_aux2_button_state == 0:

	# function to start the music box object
	# def start_music_box(self, song_number=0, tempo=160):
		# self.robot_music_box = music_box(song=songs[song_number], tempo=tempo)

	# function handles the music box workings while robot_music_box object exists
	# note frequency is constantly checked against last note freq, when a change occurs the new freq is sent to arduino
	# and last_freq is updated
	# def handle_music_box(self,):
	# 	if self.robot_music_box is not None:
	# 		f = self.robot_music_box.get_note()
	# 		if f != self.last_buzzer_freq and f != None:
	# 			self.send_message(21, [f])
	# 			self.last_buzzer_freq = f
	# 	# if the music box is on its last note, set the object to be none
	# 	if self.robot_music_box.notes == self.robot_music_box.current_note and self.robot_music_box.note_on == False:
	# 		self.robot_music_box = None


	# arduino message commands
	# <10, left_motor_val, right_motor_val>										| send motor values
	# <11>, <11,left_encoder_val, right_encoder_val>							| sends request for encoder values, encoder values are received
	# <12,left_encoder_reset, right_encoder_reset>								| resets encoder count if 1
	# <20>, <20,reset_sw_state, motor_sw_state, aux1_sw_state, aux2_sw_state>	| sends request for button states, button states are 0 or 1
	# <21, buzzer_freq>															| sets buzzer frequency
	# <22, LED_HIGH_time_ms, LED_LOW_time_ms>									| sets the LED high and low period
	# <91, kp, ki, kd>															| updates PID values
	# <92,direct_drive_if_not_zero>												| updates direct drive state
	def callback_serial_publisher(self, timerevent):
		#print('serial_pub')
		if self.motor_update_timer.check_timer():
			self.serial_publisher.publish(f'<10,{int(self.left_motor_speed)},{int(self.right_motor_speed)}>')
			self.motor_update_timer.start()

		if self.encoder_update_timer.check_timer():
			self.serial_publisher.publish(f'<11>')
			self.encoder_update_timer.start()
		
		if self.button_update_timer.check_timer():
			self.serial_publisher.publish(f'<20>')
			self.button_update_timer.start()

	
	def callback_steering_mixer(self, timerevent):
		self.left_motor_speed = constrain((self.cmd_throttle - self.cmd_rotation)*self.steering_mixgain, -100, 100)
		self.right_motor_speed = constrain((self.cmd_throttle + self.cmd_rotation)*self.steering_mixgain, -100, 100)
					
	# function to calculate the distance the left and right wheel have moved since last encoder reset
	# returns the distance traveled in inches
	def distance_moved(self):
		return (self.last_left_enc/self.ticks_per_inch), (self.last_right_enc/self.ticks_per_inch)

	# function to reset the encoder values on the arduino
	# takes a left_enc or right_enc boolean allowing for one encoder to be reset without changing the other count
	def reset_encoder_values(self, vals='{}'):
		vals = json.loads(vals)
		# send message
		left_enc = False
		right_enc = False
		if 'left' in vals:
			left_enc = vals['left']
		if 'right' in vals:
			right_enc = vals['right']
		self.serial_publisher.publish(f'<13,{left_enc},{right_enc}>')

	def send_pid_values(self, vals='{}'):
		vals = json.loads(vals)
		kp = self.kp
		ki = self.ki
		kd = self.kd
		if 'kp' in vals:
			kp = vals['kp']
		if 'ki' in vals:
			ki = vals['ki']
		if 'kd' in vals:
			kd = vals['kd']
		self.serial_publisher.publish(f'<91,{kp}1,{ki},{kd}>')

	def send_indicator_LED_values(self, vals='{}'):
		vals = json.loads(vals)
		high = 1000
		low = 500
		if 'high_ms' in vals:
			high = vals['high_ms']
		if 'low_ms' in vals:
			low = vals['low_ms']
		self.serial_publisher.publish(f'<22,{high},{low}>')

	def estop(self):
		self.cmd_throttle = 0
		self.cmd_rotation = 0

	# function to decode serial messages
	# removes formatting and isolates cmd and data
	def decode_received_serial_message(self, response):
		resp = str(response).strip('\r\n').strip('<').strip('>')
		resp = resp.split(',')
		return resp[0], resp[1:]

	def callback_serial_subscriber(self, msg):
		cmd, data = self.decode_received_serial_message(msg.data)
		print(f'cmd: {cmd} data: {data}')

		if int(cmd) == 11:
			self.last_left_enc = int(data[0])
			self.last_right_enc = int(data[1])
		elif int(cmd) == 20:
			self.last_reset_button_state = int(data[0])
			self.last_motor_button_state = int(data[1])
			self.last_aux1_switch_state = int(data[2])
			self.last_aux2_switch_state = int(data[3])
		else:
			pass

	def stop(self):
		self.estop()
		self.serial_publisher.publish(f'<10,0,0>')

if __name__ == '__main__':
	rospy.init_node("robot")

	rospy.set_param('/serialcomm/pub_freq', 50)
	rospy.set_param('/serialcomm/port', '/dev/ttyACM1')
	rospy.set_param('/serialcomm/baud', 115200)

	rospy.set_param('/robot/freqs/motor_mixer', 20)

	rospy.set_param('robot/buttons/timer_freq', 20)

	rospy.set_param('/robot/freqs/update_motor', 10)
	rospy.set_param('/robot/freqs/update_encoder', 25)
	rospy.set_param('/robot/freqs/update_button', 10)

	#create robot object
	robot = P3_DX_robot()

	rospy.on_shutdown(robot.stop)
	rospy.loginfo("P3DX maincontroller rosnode started")

	robot.start_serial_pub_timers()

	robot.send_pid_values()
	robot.send_indicator_LED_values()

	rospy.loginfo("Robot fully initialized")

	try:
		rospy.spin()
	except:
		robot.stop()
	#upodate PID variables and set robot to direct drive (without PID controllers)
	#robot.send_message(91, [robot.kp,robot.ki,robot.kd])
	# robot.send_message(92, [1])

	# #update LED intervals to 500ms HIGH and 1000ms LOW
	# robot.send_message(22, [500,1000])

	# #set run_flag as true
	# run_flag = True

	# #create socket thread with socket and robot object arguments
	# sock_thread = threading.Thread(target=socket_thread, args=(s, robot))
	# #print the socket object, start the thread and start the robot timers
	# print(sock_thread)
	# sock_thread.start()
	# robot.start_robot_update_timers()

	# #start a music box object
	# #robot.start_music_box(0, tempo=160)
	# #robot.start_music_box(2, tempo=114)
	# #robot.start_music_box(3, tempo=144)

	# #while the run_flag is true
	# while run_flag:

	# 	#if socket thread stops
	# 	if not sock_thread.is_alive():
	# 		#update speeds to 0 and force the robot to stop
	# 		robot.left_motor_speed = 0
	# 		robot.right_motor_speed = 0
	# 		robot.update_motor_speed(forced=True)
	# 		#try to reconnect if autoreconnect_socket is set
	# 		if autoreconnect_socket:
	# 			print("looking for new socket connection")
	# 			s = network_sock()
	# 			s.connect(server_ip, server_port)
	# 			print("new socket connected")
	# 			sock_thread = threading.Thread(target=socket_thread, args=(s, robot))
	# 			sock_thread.start()
	# 		#otherwise, stop the robot loop
	# 		else:
	# 			run_flag = False

	# 	#send robot messages and receive data from robot
	# 	robot.update_motor_speed()
	# 	robot.get_encoder_values()
	# 	robot.get_button_values()
	# 	robot.handle_buttons()
	# 	robot.handle_music_box()
	