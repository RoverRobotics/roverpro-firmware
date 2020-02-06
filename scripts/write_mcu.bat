set pk3cmd="C:\opt\Microchip\MPLAB IDE\Programmer Utilities\PICkit3\PK3CMD.exe"
set part=24FJ256GB106

set new_image=%~dpnx1
echo "writing %1 to device"

%pk3cmd% -P%part% -F"%new_image%" -M -L
