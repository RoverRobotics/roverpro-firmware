#!/bin/sh

#Check out repeater testing files to desktop
sudo svn co http://192.168.180.210:8080/svn/RoboteX_SVN/Firmware/Avatar/Avatar_Micro_Robot/Robot_V2_Accessories/Repeater/Testing ~/Desktop/Repeater_Test --username taylor

#Change ownership of files
sudo chown -hR taylor ~/Desktop/Repeater_Test
sudo chmod +x ~/Desktop/Repeater_Test/configure_repeater.sh

