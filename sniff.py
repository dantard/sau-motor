#!/usr/bin/env python3
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
local_ip = get_if_addr(sys.argv[1])


def bandwidth(delay):
    global total_bytes_on_wire
    while True:
        text = str()
        for k in total_bytes_on_wire:
            text += f"{k}: {total_bytes_on_wire[k] * 8 / delay / 1024:5.2f}, "
            total_bytes_on_wire[k] = 0
        # print(text, end="\r")
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
                if "ABCD" in payload_str:
                    base = payload_str.index("ABCD")
                    index = payload_str[base + 4:base + 5]
                    serial = payload_str[base + 5: base + 8]
                    serial = serial.encode('utf-8', errors='ignore')
                    serial = int.from_bytes(raw_payload[base + 5:base + 7], byteorder='little')
                    print(index, serial, ip_layer.frag)

                if "aaaa" in payload_str:
                    if ip_layer.frag == 0:
                        frag_counters[0] = 0
                        serial = int.from_bytes(raw_payload[68:73], byteorder='little')
                        serials[0] = serial


                    else:
                        frag_counters[0] += 1
                    # print("LONG", serials[0], frag_counters[0])

                elif "bbb" in payload_str:
                    if ip_layer.frag == 0:
                        frag_counters[1] = 0
                        serial = int.from_bytes(raw_payload[68:73], byteorder='big')
                        serials[1] = serial
                    else:
                        frag_counters[1] += 1
                    # print("SHOOOOOOORT", serials[1], frag_counters[1])

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
threading.Thread(target=bandwidth, args=(1,)).start()

try:
    # Capture packets on the specified interface
    sniff(iface=sys.argv[1], prn=packet_handler, store=False)
except Scapy_Exception as e:
    print(f"Error capturing packets: {e}")
except PermissionError:
    print("Permission denied: run the script as root or administrator.")
