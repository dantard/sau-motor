#source /opt/ros/iron/setup.bash

ros2 daemon stop

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/danilo/work/DDS/cyclone.xml

#export RMW_IMPLEMENTATION=rmw_connextdds
#export NDDS_DISCOVERY_PEERS=udpv4://192.168.1.1
#export NDDS_TRANSPORT_UDPv4_INTERFACE=192.168.1.1

#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTRTPS_DEFAULT_PROFILES_FILE=/home/danilo/work/DDS/fastrtps_profiles.xml

source /opt/ros/iron/setup.bash
source /home/danilo/work/sparrow_ws/install/setup.bash

ros2 daemon start