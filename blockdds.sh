#!/bin/bash

sudo iptables -A OUTPUT -p udp --dport 7400:7800 -o $1 -j REJECT
sudo iptables -A OUTPUT -p tcp --dport 25000:26000 -o $1 -j REJECT
