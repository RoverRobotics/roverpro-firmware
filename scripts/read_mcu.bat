set pk3cmd=C:\opt\Microchip\MPLAB IDE\Programmer Utilities\PICkit3\PK3CMD.exe
set part=24FJ256GB106
For /f "tokens=1-3 delims=/:." %%a in ("%TIME%") do (set mytime=T%%a%%b%%c)

set scriptdir=%~dp0
set buildbase=%scriptdir%../build
set builddir=%buildbase%/ipe
mkdir "%builddir%" 2>  NUL

PUSHD "%builddir%"
set old_image=%builddir%/memory-%mytime%.hex
%pk3cmd% -P%part% -GF "%old_image%" -L
POPD
