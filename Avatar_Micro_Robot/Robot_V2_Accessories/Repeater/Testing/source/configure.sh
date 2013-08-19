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

max_repeaters=50
num_repeaters=50
MAC_range=${new_MAC:0:14}

#echo --$new_SSID--
#echo --$new_MAC--

#MAC=$MAC_range:01
MAC=$new_MAC
#/put 


for i in `seq 1 $max_repeaters`
do
set mesh_string[$i]=""
done


for i in `seq 1 $num_repeaters`
do
  lowest_MAC_byte=`printf "%02x" $i`
  if [ "$repeater_number" = "$i" ]; then
    echo ""
  else
    mesh_string[$i]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_$i wds-address=$MAC_range:$lowest_MAC_byte"
  fi
 done

ssh -o "StrictHostKeyChecking no" -o "ServerAliveInterval=5" -o "ServerAliveCountMax=1" -q -i DSA/id_dsa admin@$old_repeater_IP << ENDSSH
/put "Starting configuration"
#Change password
/password old-password="$old_password" new-password=$new_password confirm-new-password="$new_password"
#Remove bridge
/interface bridge port remove 1
#Remove wireless IP address
/ip address remove 1
#Add wireless IP address
/ip address add address 5.5.5.1 netmask 255.255.255.0 interface wlan1
#Remove all wireless settings from previous network
/interface wireless wds remove [/interface wireless wds find]
/interface mesh port remove [/interface mesh port find]
/interface mesh remove [/interface mesh find]
#Add mesh
/interface mesh add disabled=no
/interface mesh port add interface=wlan1 mesh=mesh1
#Change MAC and SSID together (so router doesn't become
#unreachable if only one succeeds
/interface wireless set wlan1 mac-address=$MAC ssid=$new_SSID
#Setup wireless interface
/interface wireless set wlan1 disabled=no frequency=$frequency band="2ghz-b/g" mode=ap-bridge wds-mode=static-mesh wds-default-bridge=mesh1 hide-ssid=yes
#Add mesh nodes
${mesh_string[1]}
${mesh_string[2]}
${mesh_string[3]}
${mesh_string[4]}
${mesh_string[5]}
${mesh_string[6]}
${mesh_string[7]}
${mesh_string[8]}
${mesh_string[9]}
${mesh_string[10]}
${mesh_string[11]}
${mesh_string[12]}
${mesh_string[13]}
${mesh_string[14]}
${mesh_string[15]}
${mesh_string[16]}
${mesh_string[17]}
${mesh_string[18]}
${mesh_string[19]}
${mesh_string[20]}
${mesh_string[21]}
${mesh_string[22]}
${mesh_string[23]}
${mesh_string[24]}
${mesh_string[25]}
${mesh_string[26]}
${mesh_string[27]}
${mesh_string[28]}
${mesh_string[29]}
${mesh_string[30]}
${mesh_string[31]}
${mesh_string[32]}
${mesh_string[33]}
${mesh_string[34]}
${mesh_string[35]}
${mesh_string[36]}
${mesh_string[37]}
${mesh_string[38]}
${mesh_string[39]}
${mesh_string[40]}
${mesh_string[41]}
${mesh_string[42]}
${mesh_string[43]}
${mesh_string[44]}
${mesh_string[45]}
${mesh_string[46]}
${mesh_string[47]}
${mesh_string[48]}
${mesh_string[49]}
${mesh_string[50]}
/interface wireless print
/put "Finished configuration"
/beep
/delay 1
/beep
/system reboot
ENDSSH
