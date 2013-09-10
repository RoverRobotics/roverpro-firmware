#!/bin/sh

# Description: This file ensures all 
# dependencies are satisfied for this
# test program by attempting to install
# everything.  This script should NOT 
# need to be run before every test.

sudo ufw default deny
sudo ufw enable
sudo apt-get update

# install the GNU C++ compiler
sudo apt-get install g++

# install libusb to enable
# communication with USB devices
sudo apt-get install libusb-dev
sudo apt-get install libusb-1.0-0-dev

# install libsdl (Simple DirectMedia Layer Library)
# to interface with an Xbox Controller
sudo apt-get install libsdl1.2-dev

# install vim text editor
sudo apt-get install vim

# install VLC media player
sudo apt-get -y install vlc-nox  # NB: -y flag automatically says yes to any prompts

# install TVP 5150 (make sure modprobe is installed first)
sudo modprobe -v tvp5150

