set pk3cmd=C:\opt\Microchip\MPLAB IDE\Programmer Utilities\PICkit3\PK3CMD.exe
set part=24FJ256GB106

set scriptdir=%~dp0
set buildbase=%scriptdir%/../build
set builddir=%buildbase%/ipe
set new_image=%~dpnx1
set range=0-83ff

mkdir "%builddir%" 2> NUL

echo "writing %1 to device"

PUSHD "%builddir%"
%pk3cmd% -P%part% -F"%new_image%" -M -Y -L

POPD
