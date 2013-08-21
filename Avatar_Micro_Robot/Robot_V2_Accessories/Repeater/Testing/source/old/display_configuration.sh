#!/bin/bash

sudo ifconfig wlan0 down
sudo ifconfig eth0 192.168.88.2 up

ssh -o "StrictHostKeyChecking no" -t admin@192.168.88.1 << ENDSSH
/ip address print
/system beep
/interface wireless wds print
/interface wireless print
ENDSSH

sudo ifconfig wlan0 up
