#!/bin/sh

#!/bin/sh

#get path of current script
shell_script=`readlink -f "$0"`

# compile the program
current_folder=`dirname $shell_script`
cd $current_folder/source
sudo make

#current_folder=`dirname $shell_script`
#cd $current_folder/source

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
current_folder=`dirname $shell_script`
cd $current_folder/source




while [ true ]
do
user_input=""
echo "\r\n\r\n\r\n"
echo "[d]isplay repeater positions"
echo "[o]pen dispenser"
echo "[c]lose dispenser"

read user_input


if [ "$user_input" = "d" ]; then
sudo ./DispenserTest d

elif [ "$user_input" = "o" ]; then
	sudo ./DispenserTest o

elif [ "$user_input" = "c" ]; then
	sudo ./DispenserTest c

else
	echo "\r\n\r\nInvalid input.  Try again.\r\n"
fi

done
