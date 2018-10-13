#!/bin/sh

sudo ufw default deny
sudo ufw enable
sudo apt-get update
sudo apt-get install g++
sudo apt-get install libusb-dev
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libsdl1.2-dev
sudo apt-get install vim

