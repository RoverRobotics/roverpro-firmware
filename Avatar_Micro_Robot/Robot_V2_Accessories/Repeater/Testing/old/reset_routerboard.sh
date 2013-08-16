#!/bin/bash

sudo ifconfig wlan0 down
sudo ifconfig eth0 192.168.88.2

ssh-keygen -R 192.168.88.1

#possible_IPs[0]="192.168.88.1"
#possible_IPs[1]="10.1.123.3"
#possible_IPs[2]="10.1.123.4"
#possible_IPs[3]="10.1.123.5"
#possible_IPs[4]="10.1.123.6"
#possible_IPs[5]="10.1.123.7"

#for i in 0 1 2 3 4 5
#do
#  current_IP=${possible_IPs[$i]}
#  computer_IP=$current_IP"9"
# echo "Trying "$current_IP" computer IP is "$computer_IP
#  sudo ifconfig wlan0 down
#  sudo ifconfig eth0 $computer_IP
#  #ping_results=`ping -c 1 $current_IP | grep "bytes from"`
#  ping_results=`ping -c 1 $current_IP`
#  echo "Ping results are: "$ping_results#

#  if test -n "$ping_results"; then
#    echo "IP is "$current_IP
#    ssh -t admin@$$current_IP << ENDSSH
#      echo Starting
#      /system reset-configuration
#      /system reboot
#ENDSSH
#break
#  fi
#done

  

ssh -o "StrictHostKeyChecking no" -t admin@192.168.88.1 << ENDSSH
echo Starting
/system reset-configuration
/system reboot
ENDSSH

ssh-keygen -R 192.168.88.1

sudo ifconfig wlan0 up

