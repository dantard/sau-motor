#!/usr/bin/env python3
import argparse
import socket
import struct
import sys
import time
import signal
from statistics import mean, median

import numpy as np
from matplotlib import pyplot as plt
from numpy import arange, std
import matplotlib.pyplot as mpl

mpl.rcParams['font.size'] = 12
mpl.rcParams["font.family"] = "monospace"

keep_going = True

server_address = None


def ctrl_c(s, f):
    global keep_going
    keep_going = False
    sck = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sck.sendto(bytes(), server_address)


parser = argparse.ArgumentParser(
    prog='Delay Watcher',
    description='Whatches Delays!')

parser.add_argument('-a', '--address', type=str, required=True)
parser.add_argument('-w', '--write', default=None, type=str)
parser.add_argument('-x', '--plot', default=None, nargs='+')
parser.add_argument('-l', '--preview', action='store_true')
parser.add_argument('-d', '--discard', default=0, type=float)
parser.add_argument('-c', '--cut', default=1e20, type=float)
parser.add_argument('-P', '--print', action='store_true')

args = parser.parse_args()

if args.plot is not None:
    for file in args.plot:
        with open(file, "r") as f:
            data = []
            t = []
            for line in f:
                t_, data_ = line.split(",")
                t.append(float(t_))
                data.append(float(data_))

            data = list(zip(t, data))
            data = [d for d in data if args.discard < d[0] < args.cut]
            t, data = zip(*data)

            t = [x - t[0] for x in t]

            print(mean(data), std(data), median(data), max(data), min(data))

            plt.scatter(t, data, 4, c='r', zorder=2)
            plt.plot(t, data, zorder=1)
            plt.xticks(arange(0, max(t), 0.5))
            plt.xlabel("Time (s)")
            plt.ylabel("Delay (s)")
            plt.yticks([0.0041, 0.015, 0.15])
            plt.legend(["Message delay"], loc='upper right')

    if args.print:
        plt.savefig(args.plot[0].replace(".csv", ".pdf"), format="pdf", bbox_inches="tight")
        print("Saved to", args.plot[0].replace(".csv", ".pdf"))

    plt.show()

    sys.exit(0)

signal.signal(signal.SIGINT, ctrl_c)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the address and port
address, port = args.address.split(":")
server_address = (address, int(port))
print('Starting up on %s port %s' % server_address)
server_socket.bind(server_address)

storage = {}
count = 0

while keep_going:
    # Wait for a message
    # print('Waiting for a message...')
    data, client_address = server_socket.recvfrom(233)  # Receive 8 bytes (two integers)
    if len(data) == 0:
        continue

    int_values = struct.unpack('!iiq', data)
    # print('Received %s bytes from %s: %s' % (len(data), client_address, int_values))
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

if args.write is not None:
    print("Writing to", args.write)
    with open(args.write, "w") as f:
        for t, data in zip(t, data):
            f.write(f"{t}, {data}\n")

if args.preview:
    t = [x - t[0] for x in t]
    plt.scatter(t, data, 2)
    plt.xticks(arange(0, int(t[-1]), 0.5))
    plt.plot(t, data)
    plt.show()
