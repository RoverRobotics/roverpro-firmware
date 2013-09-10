boom_mcu_id=`lsusb | grep -o "2694:0012"`
boom_empia_id=`lsusb | grep -o '2694:0013'`

echo "\r\n\r\n\r\n"

if test -n "$boom_empia_id"; then
echo "boom empia chip found"
else
echo "boom empia chip NOT found"; 
fi

if test -n "$boom_mcu_id"; then
echo "boom mcu found"
else
echo "boom mcu NOT found";
fi
