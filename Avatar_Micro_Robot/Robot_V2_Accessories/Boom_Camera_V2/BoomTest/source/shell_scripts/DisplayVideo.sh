#!/bin/sh
# Description: Configures VLC player and brings up the
# camera video from USB hub

# HACK: We have to make the hub full-speed for PIC USB communication, but 
# video is high-speed???
# Re-scan for the device we previously disabled
sudo su -c "echo -n '1' > /sys/bus/pci/rescan"

sleep 5 # wait some time for the USB devices to disappear

# Re-install the driver with proper configuration
# (since conflicts with arm test driver installation)
echo "Uninstalling em28xx driver..."
sudo modprobe -r em28xx
echo "\t em28xx uninstalled."
echo "Installing em28xx driver..."
sudo modprobe em28xx card=65
echo "\t em28xx installed."

# tell the driver to look for and attach to our empia chip
sudo su -c "echo '2694 0013' > /sys/bus/usb/drivers/em28xx/new_id"
sleep 5  # wait for the empia chip to initialize

# Launch VLC player
# NB: assumes only one camera is plugged in
#     and on a netbook with a built-int camera (video0)
echo "Launching VLC player..."
cvlc v4l2:///dev/video1:standard=ntsc:width=640:height=480
