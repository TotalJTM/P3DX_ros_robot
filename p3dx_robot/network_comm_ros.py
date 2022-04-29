#! /usr/bin/python3
#   JTM 2022
#   the serial communications class wrapper for ROS

import sys
import rclpy
import rclpy.node as node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32, Bool

from p3dx_robot.network import network_sock
import json
import time

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

class Network_Comm_ROS:
	def __init__(self, nodeinst):

		super().__init__()

		self.n = nodeinst

		self.message_subscriber = self.n.create_subscription(String, 'netcomm_message_in', self.callback_send_message, 10)
		self.cmd_throttle_publisher = self.n.create_publisher(Float32, 'cmd_throttle', 1)
		self.cmd_rotation_publisher = self.n.create_publisher(Float32, 'cmd_rotation', 1)
		self.cmd_stopped_publisher = self.n.create_publisher(Bool, 'netcomm_stopped', 1)
		self.network_message_publisher = self.n.create_publisher(String, 'netcomm_message_out', 1)
		self.publisher_loop = self.n.create_timer(1.0/self.n.declare_parameter("pub_freq", 50).value, self.callback_handle_socket)
		self.trigger_close = self.n.create_service(Trigger, "netcomm_close", self.callback_stop)
		self.trigger_restart = self.n.create_service(Trigger, "netcomm_restart", self.callback_restart)

		self.net = network_sock(autoreconnect=True)
		self.connect(self.n.declare_parameter("serverip", '192.168.1.5').value, self.n.declare_parameter("serverport", 12345).value)

	def callback_handle_socket(self, event=None):
		message = self.net.receive()
		if message is not None:
			items = message.decode('UTF-8')

			items

			if items != None and items != '':
				self.n.get_logger().info(f'Network Comm handle_socket : {items}')
				try:
					d_items = json.loads(items)
				except:
					self.n.get_logger().info(f'Network Comm received unexpected data : {items}')
					return

				self.n.get_logger().info(f'Network Comm handle_socket : {d_items}')
				
				if 'msg' in items:
					s = String()
					s.data = d_items['msg']
					self.network_message_publisher.publish(s)

				if 'cmd_throttle' in items:
					d = Float32()
					d.data = float(d_items['cmd_throttle'])
					self.cmd_throttle_publisher.publish(d)

				if 'cmd_rotation' in items:
					d = Float32()
					d.data = float(d_items['cmd_rotation'])
					self.cmd_rotation_publisher.publish(d)

				if 'stop' in items:
					self.net.close()
					d = Bool()
					d.data = False
					self.cmd_stopped_publisher.publish(d)

	def callback_send_message(self, msg):
		#try:
		self.n.get_logger().info(f'sending network message {msg.data.encode()}')
		self.net.send(msg.data.encode())
		#except:
		#    rospy.loginfo(f'Message not sent: {msg}')

	def connect(self, ip, port):
		if not isinstance(self.net, network_sock):
			self.net = network_sock()
		
		self.net.connect(ip, port)
		d = Bool()
		d.data = True
		self.cmd_stopped_publisher.publish(d)
		self.n.get_logger().info("Network communication started")

	def stop(self):
		self.net.stop()

	def callback_stop(self):
		self.stop()

	def callback_restart(self):
		self.stop()
		self.connect(self.n.get_parameter("serverip").value, self.n.get_parameter("serverport").value)

def main(args=None):
	rclpy.init(args=sys.argv)
	node = rclpy.create_node("networkcomm")
	node.get_logger().info("Network Comm object started, looking for connection")
	netcomm_wrapper = Network_Comm_ROS(node)
	node.get_logger().info("Network Comm object fully initialized")

	rclpy.spin(node)

	node.get_logger().info("Network Comm stopped")
	netcomm_wrapper.stop()
	rclpy.shutdown()

if __name__ == "__main__":
	main()