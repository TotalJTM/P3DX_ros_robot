#! /usr/bin/python3
#   JTM 2022
#   the serial communications class wrapper for ROS

import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32
import keyboard

class Simple_Keyboard:
	def __init__(self):

		super().__init__()

		self.keyboard_flag = True
		self.rot_mag = 10
		self.throt_mag = 30
		# self.message_subscriber = rospy.Subscriber(, String, self.callback_send_serial_message, queue_size=10)
		self.cmd_throttle_publisher = rospy.Publisher('cmd_throttle', Float32, queue_size=10)
		self.cmd_rotation_publisher = rospy.Publisher('cmd_rotation', Float32, queue_size=10)
		self.keyboard_reader_timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("/inputdev/freq", 50)), self.keyboard_input)


	def keyboard_input(self, event=None):
		while self.keyboard_flag:
			try:  # used try so that if user pressed other than the given key error will not be shown
				print(keyboard.read_key())
				if keyboard.is_pressed('q'):
					self.keyboard_flag = False
					print('Stopping keyboard input reading')
					raise KeyboardInterrupt

				if keyboard.is_pressed('w'):
					self.cmd_throttle_publisher.publish(self.throt_mag)
				elif keyboard.is_pressed('s'):
					self.cmd_throttle_publisher.publish(-self.throt_mag)
				else:
					self.cmd_throttle_publisher.publish(0.0)

				if keyboard.is_pressed('a'):
					self.cmd_rotation_publisher.publish(-self.rot_mag)
				elif keyboard.is_pressed('d'):
					self.cmd_rotation_publisher.publish(self.rot_mag)
				else:
					self.cmd_rotation_publisher.publish(0.0)

			except:
				pass  # if user pressed a key other than the given key the loop will break
		
if __name__ == "__main__":
	rospy.init_node("simplekeyboard")
	keyboard_wrapper = Simple_Keyboard()
	#rospy.on_shutdown(serial_wrapper.stop)
	rospy.loginfo("Keyboard object initialized")
	try:
		rospy.spin()
	except:
		print('Exiting')