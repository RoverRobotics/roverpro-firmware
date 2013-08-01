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
#echo "[a]rm Initiator"
#echo "[f]ire Initiator"
#echo "[o]ne shot"
echo "[t]ip burn-in"
echo "[i]nitiator burn-in"
echo "[c]ycle a number of times"
echo "fi[r]mware version"
echo "[u]sb device check"
echo "[q]uit"
read user_input




#if [ "$user_input" = "a" ]; then
#sudo ./initiator_testing a

#elif [ "$user_input" = "f" ]; then
#	echo "Ready to fire? [y/n]"
#	read user_input
#	if [ "$user_input" = "y" ]; then
#		sudo ./initiator_testing f
#	fi


#elif [ "$user_input" = "o" ]; then
#sudo ./initiator_testing o


if [ "$user_input" = "c" ]; then
	echo "Enter number of times to fire:"
	read user_input
	sudo ./initiator_testing c $user_input

elif [ "$user_input" = "t" ]; then
	sudo ./initiator_testing c 50

elif [ "$user_input" = "i" ]; then
	sudo ./initiator_testing c 50

elif [ "$user_input" = "r" ]; then
echo "\r\n\r\nFirmware Version:\r\n\r\n"
sudo lsusb -v -d 2694:0014 | grep "RoboteX" -A 2

elif [ "$user_input" = "u" ]; then
initiator_pic_id=`lsusb | grep -o "2694:0014"`
if test -n "$initiator_pic_id"; then
echo "Initiator PIC is found"
else
echo "Initiator PIC is not found"; 
fi


elif [ "$user_input" = "q" ]; then
break

else
echo "\r\n\r\nInvalid input.  Try again.\r\n"
fi

done
