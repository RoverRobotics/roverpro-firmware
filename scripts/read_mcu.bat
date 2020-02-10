set pk3cmd="C:\opt\Microchip\MPLAB IDE\Programmer Utilities\PICkit3\PK3CMD.exe"
set part=24FJ256GB106

set hex_file=%~dpnx1

%pk3cmd% -P%part% -GF"%hex_file%" -L
