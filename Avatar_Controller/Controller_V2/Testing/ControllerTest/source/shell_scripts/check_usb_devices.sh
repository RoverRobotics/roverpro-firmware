controller_pic_id=`lsusb | grep -o "2694:0001"`
front_camera_id=`lsusb | grep -o "2694:0010"`
rear_camera_id=`lsusb | grep -o "2694:0011"`

echo "\r\n\r\n\r\n"

if test -n "$controller_pic_id"; then
echo "Controller PIC is found"
else
echo "Controller PIC is not found"; 
fi

if test -n "$front_camera_id"; then
echo "Front camera is found"
else
echo "Front camera is not found"; 
fi

if test -n "$rear_camera_id"; then
echo "Rear camera is found"
else
echo "Rear camera is not found"; 
fi