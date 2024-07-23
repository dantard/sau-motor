sudo ip route del 192.168.2.2
sudo ip route del 192.168.2.4
sudo ip route del 192.168.2.0/24
sudo ip route add 192.168.2.1 dev wlo1
sudo ip route add 192.168.2.4 via 192.168.2.1