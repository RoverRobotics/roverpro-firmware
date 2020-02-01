set ipecmd=C:\opt\Microchip\MPLABX\v5.30\mplab_platform\mplab_ipe\ipecmd.exe

For /f "tokens=1-3 delims=/:." %%a in ("%TIME%") do (set mytime=T%%a%%b%%c)

set scriptdir=%~dp0
set buildbase=%scriptdir%../build
set builddir=%buildbase%/ipe
mkdir "%builddir%" 2>  NUL

PUSHD "%builddir%"
set old_image=%builddir%/memory-%mytime%.hex
%ipecmd% -P24fj256GB106 -TPPK3 -GF"%old_image%" -OL
POPD
