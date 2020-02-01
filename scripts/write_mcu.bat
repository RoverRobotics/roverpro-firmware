set ipecmd=C:\opt\Microchip\MPLABX\v5.30\mplab_platform\mplab_ipe\ipecmd.exe

set scriptdir=%~dp0
set buildbase=%scriptdir%/../build
set builddir=%buildbase%/ipe
set new_image=%~dpnx1
set range=0-83ff

mkdir "%builddir%" 2> NUL

echo "writing %1 to device"

PUSHD "%builddir%"
%ipecmd% -P24fj256gb106 -TPPK3 -F"%new_image%" -M -Y -OL
POPD
