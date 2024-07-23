#!/usr/bin/env python3
import argparse
import socket
import sys
import threading
import time

from scapy import layers
from scapy.all import sniff, Scapy_Exception
from scapy.arch import get_if_addr
from scapy.interfaces import get_if_list
from scapy.layers.inet import UDP, IP
from scapy.layers.l2 import Ether
from scapy.packet import Raw

total_bytes_on_wire = {}

parser = argparse.ArgumentParser(
    prog='Sniffer',
    description='It Sniffs!')

parser.add_argument('-w', '--write', default=None)
parser.add_argument('-B', '--bandwidth', action='store_true')
parser.add_argument('-i', '--interface', required=True)
parser.add_argument('-P', '--print', action='store_true')
args = parser.parse_args()

if args.interface not in get_if_list():
    print("Interface not found")
    sys.exit(1)

local_ip = get_if_addr(args.interface)

if args.write is None and not args.bandwidth and not args.print:
    print("You must set either write, bandwidth or print")
    sys.exit(1)

if args.write is not None:
    f = open(args.write, "w")


def bandwidth(delay):
    global total_bytes_on_wire
    while True:
        text = str()
        for k in total_bytes_on_wire:
            text += f"{k}: {total_bytes_on_wire[k] * 8 / delay / 1024:5.2f}, "
            total_bytes_on_wire[k] = 0
        print(text, end="\r")
        time.sleep(delay)


frag_counters = {0: 0, 1: 0, 2: 0}
serials = {0: 0, 1: 0, 2: 0}


# Define a packet handler function
def packet_handler(packet):
    global total_bytes_on_wire
    # print(packet, type(packet))  # Display the packet details
    packet: Ether
    # print(len(packet.payload))

    if IP in packet:
        ip_layer: IP = packet[IP]
        fragment_offset_field = ip_layer.frag
        fragment_offset = fragment_offset_field & 0x1FFF

        if packet.haslayer(Raw):  # Check if the packet has a Raw layer (payload)
            raw_payload = packet[Raw].load
            try:
                # Decode the raw payload to a string, ignoring errors

                payload_str = raw_payload.decode('utf-8', errors='ignore')
                # print(payload_str)
                if "PY" in payload_str:
                    base = payload_str.index("PY")
                    try:
                        index = int(payload_str[base + 2:base + 3])
                        serial = int(payload_str[base + 5: base + 8])
                    except:
                        print("Could not parse serial number")
                        return

                    if ip_layer.frag == 0:
                        frag_counters[index] = 0
                        serials[index] = serial
                    else:
                        frag_counters[index] += 1

                    text = "{:20f}, {:1d}, {:5d}, {:3d}, {:5d}".format(time.time(), index, serial, frag_counters[index], fragment_offset_field)

                    if args.write is not None:
                        f.write(text + "\n")

                    if args.print:
                        print(text)


            except UnicodeDecodeError:
                print("Could not decode payload")

        if ip_layer.dst == local_ip:
            key = "I:" + ip_layer.src
        else:
            key = "O:" + ip_layer.dst

        if total_bytes_on_wire.get(key, None) is None:
            total_bytes_on_wire[key] = 0

        total_bytes_on_wire[key] += len(packet.payload)

        # if UDP in packet:
        #        udp_layer:UDP = packet[UDP]
        #        #print(f"Source Port: {udp_layer.sport}, Destination Port: {udp_layer.dport}", len(udp_layer.payload))


# Specify the interface
if args.bandwidth:
    threading.Thread(target=bandwidth, args=(1,)).start()

try:
    # Capture packets on the specified interface
    sniff(iface=args.interface, prn=packet_handler, store=False)
except Scapy_Exception as e:
    print(f"Error capturing packets: {e}")
except PermissionError:
    print("Permission denied: run the script as root or administrator.")

if args.write is not None:
    f.close()
