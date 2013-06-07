#!/bin/sh


sudo ifconfig eth0 192.168.88.2

ssh -t admin@192.168.88.1 << ENDSSH
echo Starting
/system reset-configuration
/system reboot
ENDSSH

ssh-keygen -R 192.168.88.1

