#!/bin/bash

old_SSID=$1
old_MAC=$2

sudo ifconfig eth0 down
sudo ifconfig wlan0 up
sudo ifconfig wlan0 $computer_IP
sudo iwconfig wlan0 essid "$old_SSID" ap "$old_MAC"
sudo ifconfig wlan0 5.5.5.100
