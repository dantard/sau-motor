sudo ip route del 192.168.2.2
sudo ip route del 192.168.2.3
sudo ip route del 192.168.2.0/24
sudo ip route add 192.168.2.1 dev wlp2s0
sudo ip route add 192.168.2.3 via 192.168.2.1
