#!/bin/sh

# BUG ALERT: if compilation step fails, it will just run old version of code

#get path of current script
shell_script=`readlink -f "$0"`


# allow FS PIC to work through hub.  This will break HS device functionality.
# cd /sys/bus/pci/drivers/ehci_hcd/
# sudo sh -c 'find ./ -name "0000:00:*" -print| sed "s/\.\///">unbind'

# figure out which device ehci_hcd is using, and remove it
cd /sys/bus/pci/drivers/ehci_hcd
for device in *; do
  if test `echo $device | grep -c "0000:"` -eq 0; then
    echo "this is not the device we're looking for";
  else
    echo "Removing " $device
    sudo su -c "echo '1' > /sys/bus/pci/devices/$device/remove"
  fi
done

# compile the program
current_folder=`dirname $shell_script`
cd $current_folder
cd ..
#sudo make

#echo "Hit Enter"
#read dummy

user_input=""

#keep 
#if [ "$1" = "m" ]; then
#  while [ 1 ]; do

#    sudo ./ArmTest m
#    echo "Device removed.  Hit [ENTER]"
#    echo "to continue, [q] to quit"
#    read user_input

#    if [ "$user_input" = "q" ]; then
#      break
#    fi

#  done



#elif [ "$1" = "l" ]; then
#  while [ 1 ]; do
#    sudo ./ArmTest m
# done

#else
  sudo ./ArmTest $1

#fi

# run the program
#sudo ./ArmTest $1
