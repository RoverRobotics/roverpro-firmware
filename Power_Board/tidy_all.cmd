cd %~dp0
set XC16="C:\Program Files (x86)\Microchip\xc16\v1.36"
set TMPFILE=files.tmp.txt
set PATH=C:\Program Files\Git\usr\bin;%PATH%

find . -iname *.h -o -iname *.c | paste -sd " " > %TMPFILE%
set /p FILES= < %TMPFILE%
clang-format -style=file -i %FILES%

rem create a default config with clang-tidy -dump-config > .clang-tidy
clang-tidy -format-style=file -fix %FILES% -- -I./include -isystem%XC16%/include/lega-c -isystem%XC16%/support/generic/h -isystem%XC16%\support\PIC24F\h -include xc16_clang_tools_stubs.h -D__PIC24FJ256GB106__ -D__PIC24F__


del %TMPFILE%
