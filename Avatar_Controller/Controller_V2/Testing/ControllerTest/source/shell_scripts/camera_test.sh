#!/bin/sh

#get path of current script
shell_script=`readlink -f "$0"`


# compile the program
current_folder=`dirname $shell_script`
cd $current_folder
cd ..


sudo ./ControllerTest p

while [ true ]
do
user_input=""
echo "\r\n\r\n\r\n"
echo "[f]ront camera"
echo "[r]ear camera"
echo "[q]uit"
read user_input


current_folder=`dirname $shell_script`
cd $current_folder/source

if [ "$user_input" = "f" ]; then
vlc v4l2:///dev/video0


elif [ "$user_input" = "r" ]; then
vlc v4l2:///dev/video0


elif [ "$user_input" = "q" ]; then
break

else
echo "\r\n\r\nInvalid input.  Try again.\r\n"
fi

done