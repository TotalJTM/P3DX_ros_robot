#! /usr/bin/python3
#   JTM 2022
#   the serial communications class wrapper for ROS

import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import String

from serial_comm import Serial_Comm
import json

class Serial_Comm_ROS:
    def __init__(self):

        super().__init__()

        self.sc = Serial_Comm(message_type='UTF-8')
        self.connect()

        self.message_subscriber = rospy.Subscriber('serialcomm_message_out', String, self.callback_send_serial_message, queue_size=10)
        self.message_publisher = rospy.Publisher('serialcomm_message_in', String, queue_size=10)
        self.message_publisher_timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("/serialcomm/pub_freq", 50)), self.callback_get_serial_message)
        rospy.Service("serialcomm_close", Trigger, self.callback_stop)
        rospy.Service("serialcomm_restart", Trigger, self.callback_restart)


    def callback_get_serial_message(self, event=None):
        if self.sc.queue_count() > 0:
            for m in self.sc.get_queue():
                self.message_publisher.publish(m)
            self.sc.clear_queue()

    def callback_send_serial_message(self, msg):
        #try:
        self.sc.send(f'{msg.data}')
        #except:
        #    rospy.loginfo(f'Message not sent: {msg}')

    def connect(self):
        if self.sc.connect(addr=rospy.get_param("/serialcomm/port", '/dev/ttyACM1'), baud=rospy.get_param("/serialcomm/baud", 115200)):
            self.sc.start_receive_thread()
        else:
            rospy.loginfo("Serial Comm port failed to initialize")
            #raise Exception('SerialNotConnected')


    def stop(self):
        self.sc.stop()

    def callback_stop(self):
        self.stop()

    def callback_restart(self):
        self.stop()
        self.connect()

        
if __name__ == "__main__":
    rospy.init_node("serialcomm")
    serial_wrapper = Serial_Comm_ROS()
    rospy.on_shutdown(serial_wrapper.stop)
    rospy.loginfo("Serial Comm object initialized")
    try:
        rospy.spin()
    except:
        serial_wrapper.stop()