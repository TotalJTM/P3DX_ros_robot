#JTM 2021
#custom network socket class

import socket
import sys
import time

class network_sock:
    #function initializes socket object and variables
    def __init__(self, message_size=256, sock=None, autoreconnect=False):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

        self.message_size = message_size
        self.master = False
        self.clientsock = None
        self.clientaddr = None
        self.autoreconnect = autoreconnect
        self.last_host = None
        self.last_port = None

    #bind function to host a socket on the designated host and port
    #host defaults to local IP address if no arg given
    def bind(self, port, host=None):
        if host is None:
            host = self.sock.gethostbyname(self.sock.gethostname())
        #bind the socket and start 5 listeners
        self.sock.bind((host, port))
        self.sock.listen(5)
        #designate that this object is the master
        self.master = True

    #connect to a socket on another computer
    #takes a host ip and a port number
    def connect(self, host, port):
        try:
            #connect to the socket
            self.sock.connect((host, port))

            last_port = host
            last_host = port

        except:
            print("Unsuccessful connection")

    #send a message to the connected device
    #message should be an encoded string, encoding and handling of data not performed
    def send(self, msg):
        #if this is the master object, send a message using the client socket connection
        #if no connection exists, accept it now and send the message
        if self.master:
            if self.clientsock is None:
                self.clientsock, self.clientaddr = self.sock.accept()
            self.clientsock.send(msg)
        #otherwise send the message using the socket object
        else:
            try:
                self.sock.send(msg)
            except:
                print("socket connection closed")
                if self.autoreconnect:
                    while not self.sock.is_connected():
                        self.connect(self.last_host, self.last_port)
                        time.sleep(1.0)

    #receive data from the connected socket
    #message will be an encoded string, decodeing and handling of data not performed
    def receive(self):
        #if this is the master object, the client socket connection should be used to 
        #receive message_size chars
        try:
            if self.master:
                msg = self.clientsock.recv(self.message_size)
            #otherwise receive data with the socket object
            else:
                msg = self.sock.recv(self.message_size)
            #if the message has contents, return it
            if msg is not None:
                return msg
        except:
            print("socket connection closed")

    #function to close all connections
    def close(self):
        #if this is the master object, close the clientsock connection
        #make it a none type and reset address/master var
        if self.master:
            self.clientsock.close()
            self.clientsock = None 
            self.address = None 
            self.master = False
        #otherwise close the current socket connection
        else:
            self.sock.close()

    def is_connected(self):
        try:
            # this will try to read bytes without blocking and also without removing them from buffer (peek only)
            data = self.sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
            if len(data) == 0:
                return True
        except BlockingIOError:
            return False  # socket is open and reading from it would block
        except ConnectionResetError:
            return True  # socket was closed for some other reason
        except Exception as e:
            print("unexpected exception when checking if a socket is closed")
            return False
        return False