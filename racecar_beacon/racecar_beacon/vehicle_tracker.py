#!/usr/bin/env python

import socket
from struct import *

HOST = '127.0.0.255'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

