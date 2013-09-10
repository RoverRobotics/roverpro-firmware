#!/bin/sh

# BUG ALERT: if compilation step fails, it will just run old version of code

# get the path of current script
shell_script=`readlink -f "$0"`

# compile the program
currentFolder=`dirname $shell_script`
cd $currentFolder/source
sudo make

# run the desired test
while true
  do
  userInput=""
  echo "\r\n\r\n\r\n"
  echo "[c]ontrol camera"
  echo "[t]est video"
  echo "[l]ist usb devices"
  echo "[q]uit"
  read userInput


  currentFolder=`dirname $shell_script`
  cd $currentFolder/source

  if [ "$userInput" = "c" ]; then
    sh ./shell_scripts/ControlCamera.sh
  elif [ "$userInput" = "t" ]; then
    sh ./shell_scripts/DisplayVideo.sh
  elif [ "$userInput" = "l" ]; then
    sh ./shell_scripts/CheckUSBDevices.sh
  elif [ "$userInput" = "q" ]; then
    break
  else
    echo "\r\n\r\nInvalid input.  Try again.\r\n"
  fi

done



