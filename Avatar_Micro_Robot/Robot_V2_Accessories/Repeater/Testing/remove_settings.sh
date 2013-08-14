#!/bin/bash

#TODO: use wireless interface, and key/password
sudo ifconfig wlan0 down
sudo ifconfig eth0 192.168.88.2
sudo ifconfig eth0 up

ssh -o "StrictHostKeyChecking no" -t admin@192.168.88.1 << ENDSSH
/beep

#reset SSID so we'll know if something goes wrong
/interface wireless set wlan1 disabled=no ssid=RXR-ERROR0

#remove all wds entries:
/interface wireless wds remove [/interface wireless wds find]
/interface mesh port remove [/interface mesh port find]
/interface mesh remove [/interface mesh find]

/delay 1
/beep

ENDSSH
