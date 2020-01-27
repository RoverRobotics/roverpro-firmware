cmake_minimum_required(VERSION 3.12)

set(CMAKE_SYSTEM_NAME Generic) # embedded, without OS
set(CMAKE_SYSTEM_VERSION 0)

set(MICROCHIP_CPU_FAMILY PIC24F)
set(MICROCHIP_CPU_MODEL_NAME pic24fj256gb106)
set(MICROCHIP_CPU_ID 24fj256gb106)

set(CMAKE_SYSTEM_PROCESSOR ${MICROCHIP_CPU_ID})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(MCU_FAMILY PIC24F)
set(MCU_MODEL)

set(MPLABX_VER v5.30)
if(CMAKE_HOST_SYSTEM_NAME STREQUAL Windows)
  set(MPLABX_ROOT
    "C:/opt/Microchip/MPLABX/${MPLABX_VER}"
    "C:/Program Files (x86)/Microchip/MPLABX/${MPLABX_VER}")
else()
  set(MPLABX_ROOT "/opt/microchip/mplabx/${MPLABX_VER}")
endif()

set(XC16_VER v1.41)
if(CMAKE_HOST_SYSTEM_NAME STREQUAL Windows)
  # I use an alternate root so MPLAB X doesn't complain about the spaces in the path to system headers.
  set(XC16_ROOT
    "C:/opt/Microchip/xc16/${XC16_VER}"
    "C:/Program Files (x86)/Microchip/xc16/${XC16_VER}")
else()
  set(XC16_ROOT "/opt/microchip/xc16/${XC16_VER}")
endif()
find_package(XC16)

# while xc16-gcc.exe doesn't play nice with feature detection, coff-gcc and elf-gcc do.
# as per https://www.microchip.com/forums/m1099635.aspx
set(CMAKE_C_COMPILER "${XC16_elf-gcc_EXECUTABLE}")
set(CMAKE_C_COMPILER_ID XC16-gcc FORCE)

set(CMAKE_EXECUTABLE_SUFFIX_C .elf)
set(ENV{XC16_OMF} elf)

set(CMAKE_C_LINKER_WRAPPER_FLAG "-Wl,")
set(CMAKE_C_LINKER_WRAPPER_FLAG_SEP ",")
set(CMAKE_EXE_LINKER_FLAGS_INIT
  -Wl,--defsym=__ICD2RAM=0
  #__ICD2RAM is used in the linker script to provision RAM for the PICKit3. We want to keep this so we can debug a live build
  )

# note: -mafrlcsj turns off license check
set(CMAKE_C_FLAGS_INIT " -mcpu=${MICROCHIP_CPU_ID} -no-legacy-libc -mafrlcsj") #-mcpu=${MICROCHIP_CPU_ID}
set(CMAKE_C_FLAGS_DEBUG_INIT " -g -D__DEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL_INIT " -Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE_INIT " -O3 -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT " -O2 -g -DNDEBUG")

add_compile_options(
  # -mcci #todo: migrate to CCI
  -ffunction-sections
  -fno-short-double
  -Wall
  -fast-math
  -merrata=psv_trap
  -no-legacy-libc
)
