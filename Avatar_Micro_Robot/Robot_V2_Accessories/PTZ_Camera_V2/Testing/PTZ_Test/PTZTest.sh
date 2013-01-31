#!/bin/sh

# BUG ALERT: if compilation step fails, it will just run old version of code

#get path of current script
shell_script=`readlink -f "$0"`

# compile the program
current_folder=`dirname $shell_script`
cd $current_folder/source
sudo make


while [ true ]
do
user_input=""
echo "\r\n\r\n\r\n"
echo "[f]irmware version"
echo "[v]ideo test"
echo "[u]sb device check"
echo "[q]uit"
read user_input


current_folder=`dirname $shell_script`
cd $current_folder/source

elif [ "$user_input" = "f" ]; then
echo "\r\n\r\nBase Firmware Version:\r\n\r\n"
sudo lsusb -v -d 2694:0005 | grep "RoboteX" -A 2
echo "\r\n\r\nRotation Firmware Version:\r\n\r\n"
sudo lsusb -v -d 2694:000a | grep "RoboteX" -A 2

elif [ "$user_input" = "v" ]; then
sh shell_scripts/video_display.sh

elif [ "$user_input" = "q" ]; then
break

elif [ "$user_input" = "u" ]; then
sh shell_scripts/check_usb_devices.sh

elif [ a$user_input = "a" ]; then
echo "blank line"

else
echo "\r\n\r\nInvalid input.  Try again.\r\n"
fi

done
