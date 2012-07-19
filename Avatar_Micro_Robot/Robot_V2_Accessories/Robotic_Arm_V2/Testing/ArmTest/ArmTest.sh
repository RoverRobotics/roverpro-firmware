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
echo "[m]ove arm"
echo "[c]alibrate arm"
echo "[d]ebug mode"
echo "[f]irmware version"
echo "[v]ideo test"
echo "[q]uit"
read user_input


current_folder=`dirname $shell_script`
cd $current_folder/source

if [ $user_input = "m" ]; then
sh shell_scripts/handle_arm_test.sh m


elif [ $user_input = "c" ]; then
sh shell_scripts/handle_arm_test.sh c


elif [ $user_input = "d" ]; then
sh shell_scripts/handle_arm_test.sh d
	

elif [ $user_input = "f" ]; then
echo "\r\n\r\nFirmware Version:\r\n\r\n"
sudo lsusb -v -d 2694:000d | grep "RoboteX" -A 2

elif [ $user_input = "v" ]; then
sh shell_scripts/video_display.sh

elif [ $user_input = "q" ]; then
break


else
echo "\r\n\r\nInvalid input.  Try again.\r\n"
fi

done
