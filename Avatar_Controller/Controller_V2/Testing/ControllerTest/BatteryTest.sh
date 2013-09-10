#!/bin/sh

# BUG ALERT: if compilation step fails, it will just run old version of code

#get path of current script
shell_script=`readlink -f "$0"`

# compile the program
current_folder=`dirname $shell_script`
cd $current_folder/source

#change timestamps on each file, so everything compiles okay
sudo find . -exec touch {} \;

#build executable
sudo make

sudo ./ControllerTest t

user_input=""
echo "Program quit running unexpectedly.  Press enter and restart program"
read user_input
