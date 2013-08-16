#!/bin/bash


#SSID of repeater after configuration
new_SSID=$1
#new MAC to use
new_MAC=$2
#Number of repeater (current supported: 1-20)
repeater_number=$3
#Frequency of repeater
frequency=$4

old_repeater_IP=$5
new_repeater_IP=$6
old_password=""
new_password="jeUf32es2rfFE4h"

max_repeaters=20
MAC_range=${new_MAC:0:14}

#MAC=$MAC_range:01
MAC=$new_MAC
#/put 
mesh_string[0]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_2 wds-address=$MAC_range:02"
mesh_string[1]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_3 wds-address=$MAC_range:03"
ssh -o "StrictHostKeyChecking no" -o "ServerAliveInterval=5" -o "ServerAliveCountMax=1" -q -i DSA/id_dsa admin@$old_repeater_IP << ENDSSH
/put "Starting configuration"
#Change password
/password old-password="$old_password" new-password=$new_password confirm-new-password="$new_password"
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
${mesh_string[0]}
${mesh_string[1]}
/put "Finished configuration"
/beep
/delay 1
/beep
/system reboot
ENDSSH
