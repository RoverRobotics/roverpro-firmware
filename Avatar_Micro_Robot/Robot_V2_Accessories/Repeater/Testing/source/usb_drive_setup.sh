#!/bin/bash

remove_old_files () {
  rm -rf temp/temp_repeater.tar
  rm -rf temp/repeater_from_flash.tar

}

setup_flash_drive () {

  drive=$1

  if [ "$drive" = "sda" ]; then
    echo "Unable to configure primary hard drive!"
    return
  fi

  sudo umount /dev/$drive
  #sudo mkdir /mnt/repeater_flash
  #sudo mount 
  
  #sudo sfdisk 
  echo "Removing paritions"

  dd if=/dev/zero of=/dev/$drive bs=512 count=1 conv=notrunc
  
}

prepare_configuration () {
  in_file_name=$1

  cd ..

  tar -cf source/temp/temp_repeater.tar "$in_file_name" --record-size 5120
  cd source

}

write_file_to_flash () {

  drive=$1

  dd if=temp/temp_repeater.tar of=/dev/$drive bs=512 count=10 seek=100 > /dev/null 2>&1
  

}

read_file_from_flash () {

  drive=$1

  dd if=/dev/$drive of=temp/repeater_from_flash.tar bs=512 count=10 skip=100 > /dev/null 2>&1

}

verify_file () {

  drive=$1
  input_file=$2

  read_file_from_flash $drive

  #diff temp_repeater.tar repeater_from_flash.tar

  if diff temp/temp_repeater.tar temp/repeater_from_flash.tar >/dev/null ; then
    echo "Verify succeeded!"
  else
    echo "Verify failed"
  fi
}

set_config_file_name () {


  cd ..
  file_list=`ls *.cfg`
  line_count=`ls -l *.cfg | wc -l`
  cd source

  config_file_name=""

  if [ "$line_count" = "0" ]; then
    echo "Error: No config file found"
  elif  [ "$line_count" = "1" ]; then
    config_file_name=$file_list
    echo "Config file found!"
  else
    echo "Error: Multiple config files found"
  fi
  


}

drive=""

if [ "`ls /dev/sdb* 2>/dev/null`" = "" ]; then
#echo "sdb does not exist"
echo
else
  drive="sdb"
fi

if [ "`ls /dev/sdc* 2>/dev/null`" = "" ]; then
#echo "sdc does not exist"
echo
else
  drive="sdc"
fi

if [ "$drive" = "" ]; then
  echo "Flash drive not found!"
  return
fi




#setup_flash_drive


remove_old_files
set_config_file_name
if [ "$config_file_name" = "" ]; then
  echo
else
  prepare_configuration "$config_file_name"
  write_file_to_flash $drive
  read_file_from_flash $drive
  verify_file $drive
  remove_old_files
fi
