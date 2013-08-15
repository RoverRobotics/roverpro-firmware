#!/bin/bash

#SSID of repeater to connect to
old_SSID=$1
#SSID of repeater after configuration
new_SSID=$2
#Partial MAC address of repeater to connect to (assuming that $MAC_range:xx will all be in same network)
#MAC_range=$3
$old_MAC=$4
$new_MAC=$5
#Number of repeater (current supported: 1-20)
repeater_number=$6
#IP of computer on which this script is running
computer_IP=$7
#Wireless IP of repeater to connect to
old_repeater_IP=$8
#Wireless IP after configuration
new_repeater_IP=$9
#Frequency of repeater
frequency=$8
old_password="abc"
new_password="abc"
sudo ifconfig eth0 down
sudo ifconfig wlan0 up
#check_MAC_range $MAC_range
#if [ "$num_AP" = "0" ]; then
#echo "Repeater not found.  Please turn on repeater."
#return
#elif [ "$num_AP" = "1" ]; then
#echo "Repeater found!  Connecting to $old_SSID ($repeater_MAC) at $old_repeater_IP"
#else
#echo "More than one production test repeater found.  Please turn off all testing repeaters"
#return
#fi
sudo ifconfig wlan0 $computer_IP
sudo iwconfig wlan0 essid "$old_SSID" ap "$old_MAC"
#MAC_range=02:00:00:00:00
#repeater_number=1
#incoming_IP=10.1.123.1
#SSID=RXR-00000001
#frequency=2422
#num_repeaters=$6
max_repeaters=20
#MAC=$MAC_range:01
MAC=$new_MAC
#/put 
mac_string[0]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_2 wds-address=$MAC_range:02"
mac_string[1]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_3 wds-address=$MAC_range:03"
sshpass -p $old_password ssh -o "StrictHostKeyChecking no" -o "ServerAliveInterval=5" -o "ServerAliveCountMax=1" -q -i DSA/id_dsa admin@$old_repeater_IP << ENDSSH
/put Starting
#Change password
/password old-password=$old_password new-password=$new_password confirm-new-password=$new_password
#Remove bridge
/interface bridge port remove 1
#Remove all wireless settings from previous network
/interface wireless wds remove [/interface wireless wds find]
/interface mesh port remove [/interface mesh port find]
/interface mesh remove [/interface mesh find]
#Remove wireless IP address
/ip address remove 1
#Add wireless IP address
/ip address add address $new_repeater_IP netmask 255.255.255.0 interface wlan1
#Add mesh
/interface mesh add disabled=no
/interface mesh port add interface=wlan1 mesh=mesh1
#Change MAC
/interface wireless set wlan1 mac-address=$MAC
#Setup wireless interface
/interface wireless set wlan1 disabled=no ssid=$new_SSID frequency=$frequency band="2ghz-b/g" mode=ap-bridge wds-mode=static-mesh wds-default-bridge=mesh1 hide-ssid=yes
#Add mesh nodes
mac_string[0]
mac_string[1]
/put Finished
/beep
/delay 1
/beep
/system reboot
/quit
ENDSSH