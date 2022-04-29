#! /usr/bin/python3
#	JTM 2022
#	custom serial port class v3.0

from serial import Serial, serialutil
import threading
import time

class Serial_Comm:
	# initializes the serial communications object
	#	arg: message_type = 'BYTE', 'UTF-8'	| the type of message being sent, UTF-8 messages are decoded before push to queue
	def __init__(self, message_type='BYTE'):
		self.sp = None
		self.buf = bytearray()
		self.queue_in = []

		self.message_type = message_type		#'BYTE', 'UTF-8'

		self.th_flag = False
		self.sthread = None
		self.readline_loop_counter_thresh = 100
		self.lock = threading.Lock()
		# self.received_data = False

	# connect to a serial port
	# 	args: baud 		| baud rate int of serial device
	# 	args: port 		| port number of device (integer value)
	# 	args: prefix	| port address path string 
	# 	args: addr		| full port address string including number
	# 	args: addtl_pyserial_kwargs	| override default pyserial kwargs 
	def connect(self, baud=115200, addr=None, port=None, prefix='/dev/ttyACM', **addtl_pyserial_kwargs):
		try:
			if addr:
				self.sp = Serial(port=addr, baudrate=baud, write_timeout=0.0, timeout=0.00, **addtl_pyserial_kwargs)
			else:
				self.sp = Serial(port=f'{prefix}{port}', baudrate=baud, write_timeout=0.0, timeout=0.00, **addtl_pyserial_kwargs)
			# print(f'serial type: {type(self.sp)}')

			mes = ''
			while mes == '':
				mes = self.readline()
			print(mes)
			
			return True
		except serialutil.SerialException:
			if addr:
				print(f'Invalid serial port {addr}')
			else:
				print(f'Invalid serial port {prefix}{port}')
			return False

	# thread function to receive data and add it to the queue
	# no input, no output
	def th_receive(self):
		while self.th_flag:
			# print(f'serial type: {type(self.sp)}')
			mes = self.readline()

			if mes:
				#print(mes)
				if self.message_type in'UTF-8':
					try:
						self.queue_in.append(mes.decode('utf-8'))
						# self.received_data = True
					except:
						print(f'Message received {mes}')
				if self.message_type in 'BYTE':
					try:
						self.queue_in.append(mes)
						# self.received_data = True
					except:
						print(f'Message received {mes}')
			else:
				return
	
	# 
	# no input, no output
	def start_receive_thread(self):
		if self.sp is not None:
			self.th_flag = True
			self.sthread = threading.Thread(target=self.th_receive)
			self.sthread.start()

	# 
	# 	args: |
	def send_bytes(self, bytes_arr):
		if self.sp is not None:
			with self.lock:
				self.sp.write(bytes_arr)

	# 
	# 	args: |
	def send(self, msg):
		# print(msg)
		if self.sp is not None:
			if self.message_type == 'UTF-8':
				msg = str(msg).encode()
			with self.lock:
				print(msg)
				self.sp.write(msg)

	# Buffer implementation taken from
	# https://stackoverflow.com/questions/19908167/reading-serial-data-in-realtime-in-python
	def readline(self):
		if self.sp is not None:
			loop_counter = 0
			i = self.buf.find(b"\n")
			if i >= 0:
				r = self.buf[:i+1]
				self.buf = self.buf[i+1:]
				return r
			while True and loop_counter < self.readline_loop_counter_thresh:
				# print(f'serial type readline: {type(self.sp)}')
				i = max(1, min(2048, self.sp.in_waiting))
				# print(f'serial type readline after: {type(self.sp)}')
				with self.lock:
					data = self.sp.read(i)
					# print(f'serial type readline after after: {type(self.sp)}, {data}')
				i = data.find(b"\n")
				if i >= 0:
					#print(i)
					r = self.buf + data[:i+1]
					self.buf[0:] = data[i+1:]
					return r
				else:
					self.buf.extend(data)

		return None
	
	# 
	# 	returns: response string, None
	def receive(self):
		response = self.readline()
		if response:
			return response.decode('utf-8')
		else:
			return None
	
	# 
	# no input, no output
	def stop(self):
		# print('stopping')
		self.th_flag = False
		self.sp = None

	# 
	# 	returns: |
	def pop_from_queue(self):
		return self.queue_in.pop(0)

	# 
	# 	returns: |
	def pop_latest_from_queue(self):
		return self.queue_in.pop(-1)

	# 
	# 	args: |
	def clear_queue(self):
		self.queue_in.clear()

	# 
	# 	returns: |
	def queue_count(self):
		return len(self.queue_in)

	# 
	# 	returns: |
	def get_queue(self):
		return self.queue_in


if __name__ == "__main__":
	s = Serial_Comm()
	s.connect(addr='/dev/ttyACM0')
	s.start_receive_thread()
	try:
		while True:
			d = s.readline()
			if d:
				print(d)

	except KeyboardInterrupt:
		s.stop()
	