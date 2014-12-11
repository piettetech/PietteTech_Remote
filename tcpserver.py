#!/usr/bin/env python
 
import socket


TCP_IP = '192.168.1.146'
TCP_PORT = 5005
BUFFER_SIZE = 500  # Normally 1024, but we want fast response
RESPONSE_MESSAGE = '\r\n\r\n 1 Success\r\n'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

while 1:
	conn, addr = s.accept()
	print ('Connection address:', addr)
	while 1:
		data = conn.recv(BUFFER_SIZE)
		if not data: break
		print ("received data:", data)
		conn.send(RESPONSE_MESSAGE)  # echo
	conn.close()

