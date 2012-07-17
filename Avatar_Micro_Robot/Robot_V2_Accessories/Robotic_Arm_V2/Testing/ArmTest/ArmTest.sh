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
cd $current_folder/source
sudo make

# run the program
sudo ./ArmTest
