#!/bin/sh

# BUG ALERT: if compilation step fails, it will just run old version of code

#get path of current script
shell_script=`readlink -f "$0"`

# compile the program
current_folder=`dirname $shell_script`
cd $current_folder/source

#change timestamps on each file, so everything compiles okay
sudo find . -exec touch {} \;

#build executable
sudo make


while [ true ]
do
user_input=""
echo "\r\n\r\n\r\n"
echo "[f]irmware version"
echo "[c]amera test"
echo "[u]sb device check"
echo "[b]attery charging info"
echo "[q]uit"
echo "[s]hutdown"
read user_input


current_folder=`dirname $shell_script`
cd $current_folder/source


if [ "$user_input" = "c" ]; then
sh shell_scripts/camera_test.sh

elif [ "$user_input" = "b" ]; then
sudo ./ControllerTest b

elif [ "$user_input" = "g" ]; then
sudo ./ControllerTest g
	

elif [ "$user_input" = "f" ]; then
echo "\r\n\r\nFirmware Version:\r\n\r\n"
sudo lsusb -v -d 2694: | grep "RoboteX" -A 2


elif [ "$user_input" = "q" ]; then
break

elif [ "$user_input" = "u" ]; then
sh shell_scripts/check_usb_devices.sh

elif [ "$user_input" = "s" ]; then
sudo shutdown -h now

elif [ a$user_input = "a" ]; then
echo "blank line"

else
echo "\r\n\r\nInvalid input.  Try again.\r\n"
fi

done