#!/bin/bash

ssh -t admin@192.168.88.1 << ENDSSH
/interface wireless wds print
/interface wireless print
ENDSSH

sudo ifconfig wlan0 up
