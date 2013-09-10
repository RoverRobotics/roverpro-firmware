#!/bin/bash

#debug="YES"

ssh-keygen -q -R 192.168.88.1 > /dev/null 2>&1
sudo ifconfig eth0 up
sudo ifconfig wlan0 down
sudo ifconfig eth0 192.168.88.2

function set_dsa_key {
echo -e "\r\n\r\nSetting up DSA key on repeater"
ssh-keygen -q -R 192.168.88.1 > /dev/null 2>&1
sftp -o "StrictHostKeyChecking no" -q admin@192.168.88.1 << ENDSFTP
put DSA/id_dsa.pub /
ENDSFTP
ssh -o "StrictHostKeyChecking no" -q -t admin@192.168.88.1 << ENDSSH
/put Starting
/user ssh-keys import public-key-file=id_dsa.pub user=admin 
/put Finished
/beep
/delay 1
/beep
/system reboot
ENDSSH
}


function quiet_host_key_reset () {
ssh-keygen -q -R 192.168.88.1 > /dev/null 2>&1
}

function reset_key {
ssh -o "StrictHostKeyChecking no" -l admin -i /home/taylor/Desktop/keys/id_dummy.pub 192.168.88.1 << ENDSSH
/beep
/system reboot
ENDSSH
}

function reset_config {
old_password="abc"
echo -e "\r\n\r\nResetting repeater configuration"
ssh-keygen -q -R 192.168.88.1 > /dev/null 2>&1
sshpass -p $old_password ssh -o "StrictHostKeyChecking no" -q -i DSA/id_dsa admin@192.168.88.1 << ENDSSH
/system reset-configuration
ENDSSH
}

function initial_config_after_reset {
old_password="abc"
new_password="abc"
echo -e "\r\n\r\nRunning initial wired configuration on repeater"
ssh-keygen -q -R 192.168.88.1 > /dev/null 2>&1
SSID=RXR-00000000
frequency=2422
MAC_range=02:00:00:00:00
incoming_IP=5.5.5.1
repeater_number=1
MAC=$MAC_range:$repeater_number
sshpass -p $old_password ssh -o "StrictHostKeyChecking no" -q -i DSA/id_dsa admin@192.168.88.1 << ENDSSH
/put Starting
/password old-password="" new-password=$new_password confirm-new-password=$new_password
/user ssh-keys import public-key-file=id_dsa.pub user=admin 
/interface bridge port remove 1
/ip address add address $incoming_IP netmask 255.255.255.0 interface wlan1
/interface wireless set wlan1 mac-address=$MAC
/interface mesh add disabled=no
/interface mesh port add interface=wlan1 mesh=mesh1
/interface wireless set wlan1 disabled=no ssid=$SSID frequency=$frequency band="2ghz-b/g" mode=ap-bridge wds-mode=static-mesh wds-default-bridge=mesh1
/put Finished
/beep
/delay 1
/beep
/system reboot
ENDSSH
}

check_MAC_range () {
MAC_range=$1
sudo ifconfig wlan0 up
sleep 1
repeater_list=`sudo iwlist wlan0 scan | grep "$MAC_range"`
num_AP=`echo $repeater_list | wc -l`
repeater_MAC=${repeater_list:29:17}
}

initial_wireless_config () {
wireless_config "RXR-00000000" "RXR-00000001" "02:00:00:00:00" 1 \
"5.5.5.100" "5.5.5.1" "5.5.5.1" 2422
}

subsequent_wireless_config () {
wireless_config "RXR-00000001" "RXR-00000001" "02:00:00:00:00" 1 \
"5.5.5.100" "5.5.5.1" "5.5.5.1" 2422
}

parse_network_number () {
local network_number=$1
local MAC_range=$network_number
local raw_MAC_string=`printf "%08x" $network_number`
echo $raw_MAC_string
MAC_range="02"
for i in  0 2 4 6
do
MAC_range=$MAC_range:${raw_MAC_string:i:2}
done
echo $MAC_range

}

parse_MAC () {
local MAC=$1

local i=0

local network_number_hex=""
for i in 3 6 9 12
do
network_number_hex=$network_number_hex${MAC:$i:2}
done

network_number_hex_caps=${network_number_hex^^} 

#echo $MAC
network_number_decimal=`echo "ibase=16; $network_number_hex_caps" | bc`
network_number_parsed=`printf "%08d" $network_number_decimal`
repeater_number_hex=${MAC:15:2}
repeater_number_hex_caps=${repeater_number_hex^^}
repeater_number_parsed=`echo "ibase=16; $repeater_number_hex_caps" | bc`
#echo $network_number
#echo $repeater_number_hex

#if [ "$debug" = "YES" ]; then
#echo "Function: parse_MAC"
#echo "Input: $MAC"
#echo "network_number_hex: " $network_number_hex
#echo "Output: $network_number"
#fi

}


select_then_configure_repeater () {
echo "test"

}

check_MAC_and_SSID () {
local MAC=$1
local SSID=$2

parse_network_number $MAC


}

wireless_config () {
#SSID of repeater to connect to
old_SSID=$1
#SSID of repeater after configuration
new_SSID=$2
#Partial MAC address of repeater to connect to (assuming that $MAC_range:xx will all be in same network)
#MAC_range=$3
$old_MAC=$4
$new_MAC=$5
#Number of repeater (current supported: 1-20)
repeater_number=$6
#IP of computer on which this script is running
computer_IP=$7
#Wireless IP of repeater to connect to
old_repeater_IP=$8
#Wireless IP after configuration
new_repeater_IP=$9
#Frequency of repeater
frequency=$8
old_password="abc"
new_password="abc"
sudo ifconfig eth0 down
sudo ifconfig wlan0 up
#check_MAC_range $MAC_range
#if [ "$num_AP" = "0" ]; then
#echo "Repeater not found.  Please turn on repeater."
#return
#elif [ "$num_AP" = "1" ]; then
#echo "Repeater found!  Connecting to $old_SSID ($repeater_MAC) at $old_repeater_IP"
#else
#echo "More than one production test repeater found.  Please turn off all testing repeaters"
#return
#fi
sudo ifconfig wlan0 $computer_IP
sudo iwconfig wlan0 essid "$old_SSID" ap "$old_MAC"
#MAC_range=02:00:00:00:00
#repeater_number=1
#incoming_IP=10.1.123.1
#SSID=RXR-00000001
#frequency=2422
#num_repeaters=$6
max_repeaters=20
#MAC=$MAC_range:01
MAC=$new_MAC
#/put 
mac_string[0]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_2 wds-address=$MAC_range:02"
mac_string[1]="/interface wireless wds add disabled=no master-interface=wlan1 name=remote_end_3 wds-address=$MAC_range:03"
sshpass -p $old_password ssh -o "StrictHostKeyChecking no" -o "ServerAliveInterval=5" -o "ServerAliveCountMax=1" -q -i DSA/id_dsa admin@$old_repeater_IP << ENDSSH
/put Starting
#Change password
/password old-password=$old_password new-password=$new_password confirm-new-password=$new_password
#Remove bridge
/interface bridge port remove 1
#Remove all wireless settings from previous network
/interface wireless wds remove [/interface wireless wds find]
/interface mesh port remove [/interface mesh port find]
/interface mesh remove [/interface mesh find]
#Remove wireless IP address
/ip address remove 1
#Add wireless IP address
/ip address add address $new_repeater_IP netmask 255.255.255.0 interface wlan1
#Add mesh
/interface mesh add disabled=no
/interface mesh port add interface=wlan1 mesh=mesh1
#Change MAC
/interface wireless set wlan1 mac-address=$MAC
#Setup wireless interface
/interface wireless set wlan1 disabled=no ssid=$new_SSID frequency=$frequency band="2ghz-b/g" mode=ap-bridge wds-mode=static-mesh wds-default-bridge=mesh1 hide-ssid=yes
#Add mesh nodes
mac_string[0]
mac_string[1]
/put Finished
/beep
/delay 1
/beep
/system reboot
/quit
ENDSSH
}

#ssh_password_key
#initial_config_after_reset
#reset_key
#ssh_password_only

function ping_wait () {
time_before=$1
time_after=$2
IP=$3

sleep $time_before

  while [ 1 ] 
  do
    alive_string=`ping -W 1 -c 1 192.168.88.1 | grep "bytes from"`
    if [ "$alive_string" = "" ]
    then
      echo -ne "."
    else
      break
    fi
  done

sleep $time_after
}

function full_reset_and_wired_config
{
reset_config

ping_wait 5 1 192.168.88.1

set_dsa_key

ping_wait 5 1 192.168.88.1

initial_config_after_reset
}

function display_configuration {
ssh-keygen -q -R 192.168.88.1 > /dev/null 2>&1
sshpass -p abc ssh -o "StrictHostKeyChecking no" -i /home/taylor/Desktop/keys/id_dsa admin@192.168.88.1 << ENDSSH
/beep
/interface wireless print
/ip address print
/interface bridge port print
/interface wireless wds print
/interface mesh print
/interface mesh port print
ENDSSH
}

wireless_recovery () {

MAC_range=$1
sudo ifconfig wlan0 up
sleep 1
repeater_list=`sudo iwlist wlan0 scan | grep "02:0"`
#num_AP=`echo $repeater_list | wc -l`
#repeater_MAC=${repeater_list:29:17}
#echo $repeater_list
#IFS=\n
#IFS=a
local i=0
#for line in $repeater_list
echo -e "\r\nSelect network number:"
while read -r line
do


repeater[$i]=${line:19:17}
parse_MAC "${repeater[$i]}"

#network[$i]=`parse_MAC "${repeater[$i]}"`
network[$i]=$network_number_parsed
repeater_number[$i]=$repeater_number_parsed
echo "[$i]: ${network[$i]} (${repeater[$i]})"
i=$((i+1))
done <<< "$repeater_list"

read selection

SSID="RXR-${network[$selection]}"
MAC="${repeater[$selection]}"

echo "SSID is $SSID"
echo "MAC is $MAC"

wireless_config $SSID "RXR-00000001" $MAC "02:00:00:00:00" 1 \
"5.5.5.100" "5.5.5.1" "5.5.5.1" 2422

}


#full_reset_and_wired_config
#initial_wireless_config
#subsequent_wireless_config
#display_configuration
#initial_config_after_reset
#parse_network_number 99999999
#parse_MAC "02:05:f5:e0:ff:01"
wireless_recovery
