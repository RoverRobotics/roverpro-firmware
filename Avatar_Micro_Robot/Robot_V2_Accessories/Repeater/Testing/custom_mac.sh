#!/bin/bash

MAC_range=$1
repeater_number=$2

incoming_IP=$3
SSID=$4
frequency=$5

echo $incoming_IP
echo $SSID
echo $frequency

new_computer_IP=$incoming_IP9 

lowest_MAC_byte=`printf "%02x" $repeater_number`
MAC=$MAC_range":"$lowest_MAC_byte

echo "MAC is " $MAC

IP[0]=3
IP[1]=4
IP[2]=5
IP[3]=6


sudo ifconfig wlan0 down
sudo ifconfig eth0 192.168.88.2
sudo ifconfig eth0 up

for i in 1 2 3 4
do
  lowest_MAC_byte=`printf "%02x" $i`
  if [ "$repeater_number" = "$i" ]; then
    mesh_string[$i]=""
  else
    mesh_string[$i]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_$i wds-address=$MAC_range:$lowest_MAC_byte"
    echo ${mesh_string[$i]}
  fi
 done			

ssh -o "StrictHostKeyChecking no" -t admin@192.168.88.1 << ENDSSH
/beep
/interface bridge port remove 1
/beep
/interface mesh add disabled=no
/interface mesh port add interface=wlan1 mesh=mesh1
/interface wireless set wlan1 mac-address=$MAC
/interface wireless set wlan1 disabled=no ssid=$SSID frequency=$frequency band="2ghz-b/g" mode=ap-bridge wds-mode=static-mesh wds-default-bridge=mesh1
/beep
${mesh_string[1]}
${mesh_string[2]}
${mesh_string[3]}
${mesh_string[4]}
/ip address print
/interface wireless wds print
/interface wireless print
/system reboot
ENDSSH
#
#/ip address add address $incoming_IP netmask 255.255.255.0 interface wlan1
#/ip address remove 0

#sudo ifconfig eth0 $new_computer_IP

#ssh -t admin@$incoming_IP << ENDSSH
#/system reboot
#ENDSSH

sudo ifconfig wlan0 up
