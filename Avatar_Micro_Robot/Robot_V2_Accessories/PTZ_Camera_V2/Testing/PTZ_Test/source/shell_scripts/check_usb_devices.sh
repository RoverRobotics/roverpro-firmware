ptz_camera_id=`lsusb | grep -o '2694:0007'`
ptz_base_id=`lsusb | grep -o "2694:0005"`
ptz_rotation_id=`lsusb | grep -o "2694:000a"`

echo "\r\n\r\n\r\n"

if test -n "$ptz_camera_id"; then
echo "PTZ camera is found"
else
echo "PTZ camera is not found"; 
fi

if test -n "$ptz_base_id"; then
echo "PTZ Base PIC is found"
else
echo "PTZ Base PIC is not found"; 
fi

if test -n "$ptz_rotation_id"; then
echo "PTZ Rotation PIC is found"
else
echo "PTZ Rotation PIC is not found"; 
fi
