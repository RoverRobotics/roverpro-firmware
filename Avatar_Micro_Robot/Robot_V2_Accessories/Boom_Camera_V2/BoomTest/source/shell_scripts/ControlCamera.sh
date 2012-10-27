#!/bin/sh

# BUG ALERT: if compilation step fails, it will just run old version of code

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
#path=`readlink -f "$0"` # get the path of the current script
#current_folder=`dirname $path`
#cd $current_folder
#cd ../

sudo ~/Desktop/BoomTest/source/BoomTest $1
