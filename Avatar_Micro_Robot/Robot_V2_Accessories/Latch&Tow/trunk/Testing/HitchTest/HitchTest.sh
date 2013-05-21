#!/bin/sh

#!/bin/sh

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
echo "[b]urn in hitch"
echo "[f]irmware version"
echo "[u]sb device check"
echo "[q]uit"
read user_input


current_folder=`dirname $shell_script`
cd $current_folder/source

if [ "$user_input" = "b" ]; then
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
sudo ./hitch_testing	

elif [ "$user_input" = "f" ]; then
echo "\r\n\r\nFirmware Version:\r\n\r\n"
sudo lsusb -v -d 2694:000f | grep "RoboteX" -A 2

elif [ "$user_input" = "u" ]; then
hitch_pic_id=`lsusb | grep -o "2694:000f"`
if test -n "hitch_pic_id"; then
echo "hitch PIC is found"
else
echo "hitch PIC is not found"; 
fi


elif [ "$user_input" = "q" ]; then
break

else
echo "\r\n\r\nInvalid input.  Try again.\r\n"
fi

done
