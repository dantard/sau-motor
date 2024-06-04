sudo ip route del 192.168.1.1
sudo ip route del 192.168.1.2
sudo ip route del 192.168.1.0/24
sudo ip route add 192.168.1.2 dev wlx00c0ca32bc4b
sudo ip route add 192.168.1.1 via 192.168.1.2
