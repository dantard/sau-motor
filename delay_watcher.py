import socket
import struct
import sys
import time
import signal

import numpy as np
from matplotlib import pyplot as plt
from numpy import arange

keep_going = True


def ctrl_c(s,f):
    global keep_going
    keep_going = False

signal.signal(signal.SIGINT, ctrl_c)

# Create a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the address and port
server_address = ('192.168.130.50', 12345)
print('Starting up on %s port %s' % server_address)
server_socket.bind(server_address)

storage = {}
count = 0


while keep_going:
    # Wait for a message
    #print('Waiting for a message...')
    data, client_address = server_socket.recvfrom(233)  # Receive 8 bytes (two integers)
    int_values = struct.unpack('!iiq', data)
    #print('Received %s bytes from %s: %s' % (len(data), client_address, int_values))
    kind = int_values[0]
    serial = int_values[1]
    timestamp = int_values[2]

    if storage.get(serial) is None:
        storage[serial] = {}
    storage[serial][kind] = time.time()
    if kind == 1:
        print(serial, storage[serial].get(1, 1), (storage[serial].get(1, 1) - storage[serial].get(0, 0)))

data = []
t = []
for k in list(storage.keys()):

    if storage[k].get(1, -1) > 0 and storage[k].get(0, -1) > 0:
        diff = (storage[k].get(1, 1) - storage[k].get(0, 0))
        data.append(diff)
        t.append(storage[k].get(0, 0))
t = [x - t[0] for x in t]
print(t)
plt.scatter(t[10:], data[10:],2)
plt.xticks(arange(0, int(t[-1]), 0.5))
#counts, bins = np.histogram(data, bins=100)
#plt.stairs(counts, bins)
plt.show()



