#!/usr/bin/env python
from socket import *

HOST = '127.0.0.1'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

s = socket(AF_INET, SOCK_STREAM)
s.connect((HOST, PORT)) # connect to server (block until accepted)
data = "Hello Server!"
s.send(data.encode())
data = s.recv(1024) # receive the response
print(data) # print the result
s.close() # close the connection
