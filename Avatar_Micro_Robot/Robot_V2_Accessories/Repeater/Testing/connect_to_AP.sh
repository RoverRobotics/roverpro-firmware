#!/bin/bash

SSID=$1
MAC=$2
source_IP=$3


echo "Parameters: " $SSID $MAC $source_IP

sudo ifconfig wlan0 up
sudo ifconfig wlan0 $source_IP

for i in 0 1; do

  echo "‎i is $i"
  up_line=$(iwconfig wlan0 | grep "$MAC")

  if test -n "$up_line"; then
  echo "Connected"
  else
  echo "Not connected.  Connecting now."

  sudo iwconfig wlan0 essid "$SSID" ap $MAC
  fi
  sleep 1

done


