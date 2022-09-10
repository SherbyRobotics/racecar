#!/usr/bin/env python
from socket import *
import struct

format_UDP= ">fffI"

HOST = '192.168.10.255'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65431

client = socket(AF_INET, SOCK_DGRAM) #UDP
client.setsockopt(SOL_SOCKET, SO_BROADCAST,1)

client.bind((HOST, PORT)) # connect to server (block until accepted)

while True:
    data = client.recvfrom(1024)
    data = struct.unpack(format_UDP,data)
    print('x: '+ str(data[0]) +' y: '+ str(data[1]) +' Theta: '+ str(data[2]) +'ID: '+ hex(data[3]))