set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 0)

set(XC16 "C:/Program Files (x86)/Microchip/xc16/v1.41")
set(CMAKE_SYSTEM_PREFIX_PATH ${XC16})

# while xc16-gcc.exe doesn't play nice with feature detection, coff-gcc and elf-gcc do.
# as per https://www.microchip.com/forums/m1099635.aspx
# set(CMAKE_C_COMPILER "${XC16}/bin/bin/coff-gcc.exe")
set(CMAKE_EXECUTABLE_SUFFIX_C .elf)
set(CMAKE_C_COMPILER   "${XC16}/bin/bin/elf-gcc.exe")
set(CMAKE_CXX_COMPILER "") # XC16 does not support C++

set(CMAKE_C_STANDARD 99)

set(CMAKE_EXECUTABLE_FORMAT elf)

# note: to enable higher optimization levels in unlicensed XC16,
# see https://github.com/cv007/XC3216/blob/master/xc32xc16-info-rev6.txt
string(APPEND CMAKE_C_FLAGS_INIT                " -no-legacy-libc")
string(APPEND CMAKE_C_FLAGS_DEBUG_INIT          " -g -D__DEBUG")
string(APPEND CMAKE_C_FLAGS_MINSIZEREL_INIT     " -Os -DNDEBUG")
string(APPEND CMAKE_C_FLAGS_RELEASE_INIT        " -O3 -DNDEBUG")
string(APPEND CMAKE_C_FLAGS_RELWITHDEBINFO_INIT " -O2 -g -DNDEBUG")

set(CMAKE_C_LINKER_WRAPPER_FLAG "-Wl,")
set(CMAKE_C_LINKER_WRAPPER_FLAG_SEP ",")

add_link_options(LINKER:--defsym=__MPLAB_BUILD=1,--gc-sections,--report-mem)

include_directories(SYSTEM
  "${XC16}/include"
  "${XC16}/support/generic/h"
  "${XC16}/support/PIC24F/h"
)

add_compile_options(
  -mcpu=24FJ256GB106
  -no-legacy-libc
  -ffunction-sections
  -fno-short-double
  -Wall
  -fast-math
  -g
)

set(CMAKE_FIND_ROOT_PATH ${XC16_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
