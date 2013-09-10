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
sudo apt-get -y install g++

# install libusb to enable
# communication with USB devices
sudo apt-get -y install libusb-dev
sudo apt-get -y install libusb-1.0-0-dev

# install libsdl (Simple DirectMedia Layer Library)
# to interface with an Xbox Controller
sudo apt-get -y install libsdl1.2-dev

# install vim text editor
sudo apt-get -y install vim

#install SVN
sudo apt-get -y install subversion

#install VLC
sudo apt-get -y install vlc

#check out firmware repository
sudo mkdir /svn
sudo chmod 577 /svn
cd /svn
svn co http://192.168.180.211:8080/svn/RoboteX_SVN/Firmware --username taylor

