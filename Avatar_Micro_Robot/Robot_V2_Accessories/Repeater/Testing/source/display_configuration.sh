#!/bin/bash

repeater_IP=$1

ssh -o "StrictHostKeyChecking no" -o "ServerAliveInterval=5" -o "ServerAliveCountMax=1" -q -i DSA/id_dsa admin@$repeater_IP << ENDSSH
/put "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
/put ""
/put "Output of /interface wireless wds print:"
/interface wireless wds print
/put "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
/put ""
/put "Output of /interface wireless print:"

/interface wireless print
ENDSSH
