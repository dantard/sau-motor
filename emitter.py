#!/bin/env python
import argparse
import os.path
import signal
import socket
import statistics
import struct
import sys
import time
from random import randint, random
import csv

import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from basic_node import BasicNode
from std_msgs.msg import String, Int64, Int64MultiArray
import os

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class Emitter(Node):

    # sudo ip route add 192.168.1.3 via 192.168.1.2
    # sudo ip route del 192.168.1.3 via 192.168.1.2
    def __init__(self, kind, idx=1, period=0.1, qos_profile=1, data_size=0, server_address=None):
        super().__init__('Emitter' + str(idx))

        self.kind = kind
        self.idx = idx
        self.publisher = self.create_publisher(self.kind, 'out' + str(idx), qos_profile)
        self.count = 0
        self.server_address = server_address
        self.data = []  # [num for _ in range(data_size)]
        self.data_size = data_size
        self.create_timer(period, self.timer_callback)
        self.prev_sent_ts = 0
        text = "ABCD{:1d}".format(self.idx, self.count)
        self.num = 0
        for i, c in enumerate(text):
            self.num = ord(c) << (i + 3) * 8 | self.num

    def timer_callback(self):
        num = self.num + self.count
        self.data = [num for _ in range(self.data_size)]
        self.send([self.count] + self.data)
        self.count += 1

    def send(self, value):
        #        diff = time.time_ns() // 1000 - self.prev_sent_ts
        #        self.prev_sent_ts = time.time_ns() // 1000
        msg = self.kind()
        if self.server_address is not None:
            message = struct.pack('!iiq', 0, value[0], time.time_ns())
            client_socket.sendto(message, self.server_address)
            # time.sleep(0.01)
        msg.data = value
        self.publisher.publish(msg)


class Receiver(Node):
    # sudo ip route add 192.168.1.1 via 192.168.1.2
    # sudo ip route del 192.168.1.1 via 192.168.1.2
    # sudo sysctl -w net.ipv4.ip_forward=1
    def __init__(self, kind, idx=1, qos_profile=1, server_address=None):
        super().__init__('Receiver' + str(idx))
        self.kind = kind
        self.subscription = self.create_subscription(self.kind, 'out' + str(idx), self.receive, qos_profile)
        self.server_address = server_address

        self.prev_ts = time.time()
        self.prev_serial = None
        self.iats = []
        self.ts = []
        self.lost = 0

    prev_sent_ts = 0

    def receive(self, value):
        now = time.time()
        serial = value.data[0]
        if self.server_address is not None:
            message = struct.pack('!iiq', 1, serial, time.time_ns())
            client_socket.sendto(message, self.server_address)

        print(
            "iat: {:4.4f} size: {:5d} serial: {:5d} sent: {:5d} lost: {:5d}".format(now - self.prev_ts, len(value.data) * 4, serial, len(self.iats), self.lost))
        if self.prev_serial is not None:
            if serial < self.prev_serial:
                self.lost = 0
            else:
                self.iats.append((now, now - self.prev_ts))
                self.lost += serial - self.prev_serial - 1

        self.prev_ts = now
        self.prev_serial = serial

    def get_lost(self):
        return self.lost

    def get_iats(self):
        return self.iats


def plot_files(files, nsigma, title=""):
    to_plot = []
    for csv_file in files:
        ts, data = [], []
        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)
            for row in reader:
                ts.append(float(row[0]))
                data.append(float(row[1]))

            # ts = [t - ts[0] for t in ts]
            # data = [d*10000 for d in data]
            sigma = statistics.stdev(data)
            mean = statistics.mean(data)

            print("Statistics", mean, sigma)
            # if hist:
            # counts, bins = np.histogram(data, bins=100000)
            # print(counts, bins)
            # plt.stairs(counts, bins)
            # plt.hist([d*10000 for d in data], bins=range(int(mean - delta), int(mean + delta)), edgecolor='black', alpha=0.7)
            increment = (mean + nsigma * sigma - (mean - nsigma * sigma)) / 250
            bins = np.arange(mean - nsigma * sigma, mean + nsigma * sigma, increment)

            # plt.hist(data, bins=bins, edgecolor='black', alpha=0.7)
            to_plot.append((ts, data, csv_file))

    min_ts = 1e12

    for ts, data, file in to_plot:
        min_ts = min(min_ts, ts[0])

    for ts, data, file in to_plot:
        ts = [t - min_ts for t in ts]
        #        plt.plot(ts, data)
        plt.scatter(ts, data, 2)

    plt.title(title)
    plt.legend(files)
    plt.show()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        prog='ProgramName',
        description='What the program does',
        epilog='Text at the bottom of help')

    parser.add_argument('-p', '--period', default=None)
    parser.add_argument('-n', '--frames-count', default=1000, type=int)
    parser.add_argument('-l', '--preview', action='store_true')
    parser.add_argument('-q', '--queue', default=None, type=int)
    parser.add_argument('-P', '--policy', default=None, type=float)
    parser.add_argument('-d', '--size', default=512, type=int)
    parser.add_argument('-t', '--title', default="receiver")
    parser.add_argument('-x', '--plot', default=None, nargs='+')
    parser.add_argument('-w', '--write', default=None)
    parser.add_argument('-i', '--idx', default=1, type=int)
    parser.add_argument('-f', '--force', action='store_true')
    parser.add_argument('-o', '--sigma', default=5, type=int)
    parser.add_argument('-s', '--port', type=int, default=-1)

    index = sys.argv.index("--ros-args") if "--ros-args" in sys.argv else len(sys.argv)
    mine = sys.argv[1:index]
    args = parser.parse_args(mine)

    if args.plot is not None:
        plot_files(args.plot, args.sigma)
        return

    if args.write:
        rmw = os.environ.get('RMW_IMPLEMENTATION', "UNKNOWN").replace("rmw_fastrtps_cpp", "fast").replace("rmw_connext_cpp", "connext").replace(
            "rmw_cyclonedds_cpp", "cyclone")
        filename = args.write + "_" + rmw + "_" + str(args.idx) + ".csv"
        if os.path.exists(filename) and not args.force:
            print("File already exists, continue? (Y/n)")
            inp = input()
            if inp != 'y' and inp != '':
                return

    if args.preview and not args.write:
        print("You must set write option to preview")
        return

    if args.policy is not None and args.queue is not None:
        print("You can't set both policy and queue size")
        return
    elif args.policy is None and args.queue is None:
        print("You must set either policy (-P) or queue size (-q)")
        return
    elif args.policy is not None:
        print("Policy: ", args.policy)
        profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=Duration(nanoseconds=args.policy * 1e9),
            # lifespan=Duration(nanoseconds=args.policy*1e9),
            depth=1
        )
    else:
        profile = args.queue

    if args.port != -1:
        server_address = ('192.168.130.50', args.port)
    else:
        server_address = None
    if args.period is not None:
        node = Emitter(Int64MultiArray, args.idx, args.period, qos_profile=profile, data_size=args.size, server_address=server_address)
    else:
        node = Receiver(Int64MultiArray, args.idx, qos_profile=profile, server_address=server_address)

    while rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.01)
        except:
            break
        if isinstance(node, Receiver) and len(node.get_iats()) > args.frames_count:
            break

    if isinstance(node, Receiver):
        if len(node.get_iats()) > 2:
            data = node.get_iats()[2:]

            if args.write:
                with open(filename, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    for ts, iat in data:
                        writer.writerow([ts, iat])
                print("Written to file: ", filename)

            if args.preview:
                plot_files([filename], args.sigma)

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
