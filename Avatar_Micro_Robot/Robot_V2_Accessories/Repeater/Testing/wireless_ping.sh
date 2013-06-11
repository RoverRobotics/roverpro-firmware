#!/bin/bash

SSID=$1
MAC=$2
source_IP=$3
destination_IP=$4
num_seconds=$5

echo "Parameters: " $SSID $MAC $source_IP $destination_IP $num_seconds

#while [ 1 ]; do

sudo ifconfig wlan0 up
sudo ifconfig wlan0 $source_IP

for i in `seq 1 $num_seconds`; do

  echo "‎i is $i"

  up_line=$(iwconfig wlan0 | grep "$MAC")

  if test -n "$up_line"; then
  echo "Connected"
  else
  echo "Not connected.  Connecting now."

  sudo iwconfig wlan0 essid "$SSID" ap $MAC
  fi
  sleep 1



  #iwconfig

  ping_string=$(ping -c 1 -W 1 $destination_IP)
  ping_string_grep=$(echo $ping_string | grep "bytes from")
 
  echo $ping_string
  if test -n "$ping_string_grep"; then
    echo "Ping succeeded.  Exiting"
    break
  else
    iwconfig wlan0 | grep "ESSID" -A 1
    echo "Ping failed.  Trying again."
  fi
  
done

#possible_IPs[0]="10.1.123.3"
#possible_IPs[1]="10.1.123.4"
#possible_IPs[2]="10.1.123.5"
#possible_IPs[3]="10.1.123.6"
#possible_IPs[4]="10.1.123.7"
#
#for i in 0 1 2 3 4 4
#do
#  current_IP=${possible_IPs[$i]}
#  #echo "Trying"$current_IP
#  sudo ifconfig wlan0 up
#  sudo ifconfig wlan0 10.1.123.100
#  sudo iwconfig wlan0 essid "$SSID"
#  
#  #while
##  #sleep 1
 # ping_string=`ping -c 1 -W 1 $current_IP | grep "bytes from"`
#  #ping_string=`ping -c 1 $current_ip`
#  #echo "ping is "$ping_string
# #ping -c 1 $current_ip
#  if test -n "$ping_string"; then
#    echo "$current_IP found!"
#  else
#    echo "Error: $current_IP not found"
#  fi
#done



