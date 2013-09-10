#!/bin/sh

#since we have to make the hub FS, try to make it HS again
#ls /sys/bus/pci/devices
#rescan for the device we previously disabled
sudo su -c "echo -n '1' > /sys/bus/pci/rescan"

#wait some time for the USB devices to disappear
sleep 2

num_devices_found=0
one=1

#wait until both USB devices are found again
while test $num_devices_found -le 1; do

num_devices_found=0

arm_camera_id=`lsusb | grep -o '2694:0008'`
arm_pic_id=`lsusb | grep -o "2694:000d"`

if test -n "$arm_camera_id"; then
echo "arm camera is found"
num_devices_found=$(($num_devices_found+$one)); else
echo "arm camera is not found"; 
fi

if test -n "$arm_pic_id"; then
echo "arm PIC is found"
num_devices_found=$(($num_devices_found+$one)); else
echo "arm PIC is not found"; 
fi

sleep 1
echo ""

done

sudo modprobe -r em28xx
echo "modprobe finished"
sudo modprobe em28xx card=65
echo "em28xx configured"
sudo su -c "echo '2694 0008' > /sys/bus/usb/drivers/em28xx/new_id"
echo "Custom VID and PID enabled"
#wait for the camera to be recognized
sleep 5
vlc v4l2:///dev/video1:standard=ntsc:width=640:height=480
