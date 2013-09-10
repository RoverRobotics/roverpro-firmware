arm_camera_id=`lsusb | grep -o '2694:0008'`
arm_pic_id=`lsusb | grep -o "2694:000d"`

echo "\r\n\r\n\r\n"

if test -n "$arm_camera_id"; then
echo "arm camera is found"
else
echo "arm camera is not found"; 
fi

if test -n "$arm_pic_id"; then
echo "arm PIC is found"
else
echo "arm PIC is not found"; 
fi
