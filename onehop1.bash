sudo ip route del 192.168.1.2
sudo ip route del 192.168.1.3
sudo ip route del 192.168.1.0/24
sudo ip route add 192.168.1.2 dev wlx00c0ca8f2998
sudo ip route add 192.168.1.3 via 192.168.1.2
