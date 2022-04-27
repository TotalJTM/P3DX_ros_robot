#! /usr/bin/python3
#   JTM 2022
#   the serial communications class wrapper for ROS

import sys
import rclpy
import rclpy.node as node
from std_srvs.srv import Trigger
from std_msgs.msg import String

from p3dx_robot.serial_comm import Serial_Comm
import json

class Serial_Comm_ROS:
	def __init__(self, nodeinst):

		super().__init__()

		self.n = nodeinst

		self.sc = Serial_Comm(message_type='UTF-8')
		self.connect()

		self.message_subscriber = self.n.create_subscription(String, 'serialcomm_message_out', self.callback_send_serial_message, 10)
		self.message_publisher = self.n.create_publisher(String, 'serialcomm_message_in', 10)
		self.message_publisher_timer = self.n.create_timer(1.0/self.n.declare_parameter("pub_freq", 50).value, self.callback_get_serial_message)
		self.trigger_close = self.n.create_service(Trigger, "serialcomm_close", self.callback_stop)
		self.trigger_restart = self.n.create_service(Trigger, "serialcomm_restart", self.callback_restart)


	def callback_get_serial_message(self, event=None):
		if self.sc.queue_count() > 0:
			msg = String()
			for m in self.sc.get_queue():
				msg.data = m
				self.message_publisher.publish(msg)
			self.sc.clear_queue()

	def callback_send_serial_message(self, msg):
		#try:
		self.n.get_logger().info(f'sending serial message {msg.data}')
		self.sc.send(f'{msg.data}')
		#except:
		#    rospy.loginfo(f'Message not sent: {msg}')

	def connect(self):
		if self.sc.connect(addr=self.n.declare_parameter("port", '/dev/ttyACM0').value, baud=self.n.declare_parameter("baud", 115200).value):
			self.sc.start_receive_thread()
			self.n.get_logger().info("Serial port connected")
		else:
			self.n.get_logger().info("Serial Comm port failed to initialize")
			#raise Exception('SerialNotConnected')


	def stop(self):
		self.sc.stop()

	def callback_stop(self):
		self.stop()

	def callback_restart(self):
		self.stop()
		self.connect()

def main(args=None):
	rclpy.init(args=sys.argv)
	node = rclpy.create_node("serialcomm")
	serial_wrapper = Serial_Comm_ROS(node)
	node.get_logger().info("Serial Comm object initialized")

	rclpy.spin(node)

	node.get_logger().info("Serial Comm stopped")
	serial_wrapper.stop()
	rclpy.shutdown()

if __name__ == "__main__":
	main()