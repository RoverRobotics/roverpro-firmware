#!/bin/bash

MAC[0]=$1
MAC[1]=$2
MAC[2]=$3
MAC[3]=$4

incoming_IP=$5
SSID=$6
frequency=$7

echo $incoming_IP
echo $SSID
echo $frequency

IP[0]=3
IP[1]=4
IP[2]=5
IP[3]=6


sudo ifconfig wlan0 down
sudo ifconfig eth0 192.168.88.2
sudo ifconfig eth0 up

for i in 0 1 2 3
do
  if [ "${MAC[$i]}" = "0" ]; then
    mesh_string[$i]=""
  else
    mesh_string[$i]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_${IP[$i]} wds-address=${MAC[$i]}"
  fi
  echo ${mesh_string[$i]}
 done			

echo "done with loop"	

ssh -t admin@192.168.88.1 << ENDSSH
echo Starting
/interface bridge port remove 1
/ip address add address $incoming_IP netmask 255.255.255.0 interface ether1
/interface mesh add disabled=no
/interface mesh port add interface=wlan1 mesh=mesh1
/interface wireless set wlan1 disabled=no ssid=$SSID frequency=$frequency band="2ghz-b/g" mode=ap-bridge wds-mode=static-mesh wds-default-bridge=mesh1
${mesh_string[0]}
${mesh_string[1]}
${mesh_string[2]}
${mesh_string[3]}
/interface wireless wds print
/interface wireless print
/system reboot
ENDSSH

sudo ifconfig wlan0 up
