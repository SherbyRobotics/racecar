#!/usr/bin/env python
from pickle import TRUE
from socket import *
import struct

format_commande = ">4s"
format_RPOS = ">fff4x"
format_OBSF = ">I4x4x4x"
format_RBID = ">I4x4x4x"

HOST = '127.0.0.1'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

s = socket(AF_INET, SOCK_STREAM)
s.connect((HOST, PORT)) # connect to server (block until accepted)

while True:
    remote_request = input("Commande: ")
    if remote_request == "close":
        break

    if remote_request == "RPOS" or remote_request == "OBSF" or remote_request == "RBID":
        remote_request_enc = bytes(remote_request, "ascii")
        remote_request_enc = struct.pack(format, remote_request_enc)
        s.send(remote_request_enc)
        
        if remote_request == "RPOS":
            recived_msgs = s.recv(1024) # receive the response
            recived_msgs = struct.unpack(format_RPOS, recived_msgs)
            msgs = recived_msgs[0]
            msgs = msgs.decode("ascii")
            print(msgs) #recived_msgs) # print the result
        
        elif remote_request == "OBSF":
            recived_msgs = s.recv(1024) # receive the response
            recived_msgs = struct.unpack(format_RPOS, recived_msgs)
            
            msgs = recived_msgs[0]
            msgs = msgs.decode("ascii")
            
            print(msgs)#recived_msgs) # print the result
        
        elif remote_request == "RBID":
            recived_msgs = s.recv(1024) # receive the response
            recived_msgs = struct.unpack(format_RPOS, recived_msgs)
            
            msgs = recived_msgs[0]
            msgs = msgs.decode("ascii")
            
            print(msgs)#recived_msgs) # print the result



    else:
        print("Commande Invalide")

s.close() # close the connection
