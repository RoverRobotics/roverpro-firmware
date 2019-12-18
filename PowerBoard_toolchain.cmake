set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 0)
set(XC16 "C:/Program Files (x86)/Microchip/xc16/v1.41")

# while xc16-gcc.exe doesn't play nice with feature detection, coff-gcc and elf-gcc do.
# as per https://www.microchip.com/forums/m1099635.aspx
# set(CMAKE_C_COMPILER "${XC16}/bin/bin/coff-gcc.exe")

set(CMAKE_C_COMPILER "${XC16}/bin/bin/elf-gcc.exe")
set(CMAKE_EXECUTABLE_FORMAT elf)

set(CMAKE_C_FLAGS_DEBUG_INIT   "-g -D__DEBUG")
set(CMAKE_C_FLAGS_RELEASE_INIT "-O3")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O1 -g -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL_INIT "-Os -DNDEBUG")

set(CMAKE_EXECUTABLE_SUFFIX_C   .elf)
set(CMAKE_EXECUTABLE_SUFFIX_CPP .elf)

set(CMAKE_EXE_LINKER_FLAGS_INIT -Wl,--defsym=__MPLAB_BUILD=1,--gc-sections,--report-mem)

set(CMAKE_C_STANDARD 99)

include_directories(SYSTEM
  "${XC16}/include"
  "${XC16}/support/generic/h"
  "${XC16}/support/PIC24F/h"
)
set(CMAKE_SYSTEM_PREFIX_PATH ${XC16})

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
