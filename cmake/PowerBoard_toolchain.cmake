cmake_minimum_required(VERSION 3.12)

set(MICROCHIP_CPU_FAMILY PIC24F)
set(MICROCHIP_CPU_MODEL_NAME pic24fj256gb106)
set(MICROCHIP_CPU_ID 24fj256gb106)

set(CMAKE_SYSTEM_NAME Generic) # embedded, without OS
set(CMAKE_SYSTEM_VERSION 0)
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

set(XC16_VER v1.50)
if(CMAKE_HOST_SYSTEM_NAME STREQUAL Windows)
  # I use an alternate root so MPLAB X doesn't complain about the spaces in the path to system headers.
  set(XC16_ROOT
    "C:/opt/Microchip/xc16/${XC16_VER}"
    "C:/Program Files (x86)/Microchip/xc16/${XC16_VER}")
else()
  set(XC16_ROOT "/opt/microchip/xc16/${XC16_VER}")
endif()
find_package(XC16)
set(CMAKE_C_OUTPUT_EXTENSION ".o")

set(TOOLCHAIN_PREFIX xc16)
#set(CMAKE_C_COMPILER "${XC16_elf-gcc_EXECUTABLE}")
set(CMAKE_C_COMPILER "${XC16_gcc_EXECUTABLE}")
#note: compiler detection detects itself as GCC and sets CMAKE_C_VERBOSE_FLAG to "-v" which causes xc16 to misbehave
# want to have compiler_id = xc16
# CMAKE_C_VERBOSE_FLAG = ""
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)


# note: CMAKE_C_FLAGS_INIT not respected for compiler detection
# these are used in compiler identification
set(CMAKE_C_FLAGS "-mcpu=${MICROCHIP_CPU_ID}")

# set(CMAKE_EXECUTABLE_SUFFIX_C .elf)

#tell all XC16 tools that we're using elf
set(ENV{XC16_OMF} elf)


set(CMAKE_C_LINKER_WRAPPER_FLAG "-Wl,")
set(CMAKE_C_LINKER_WRAPPER_FLAG_SEP ",")
# CMAKE_EXE_LINKER_FLAGS_INIT
# CMAKE_STATIC_LINKER_FLAGS_INIT

string(JOIN " " CMAKE_C_FLAGS_INIT
  # note: -mafrlcsj turns off license check
  -mafrlcsj
  "-mcpu=${MICROCHIP_CPU_ID}"
  -no-legacy-libc
  # -mcci #todo: migrate to CCI
  -fno-short-double
  -Wall
  -fast-math
  # keep build files; useful for debugging the preprocessor stage of the build
  # -save-temps
  )

set(CMAKE_C_FLAGS_DEBUG_INIT "-g -D__DEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL_INIT "-Os -DNDEBUG -ffunction-sections")
set(CMAKE_C_FLAGS_RELEASE_INIT "-O3 -DNDEBUG -ffunction-sections")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g -DNDEBUG")

foreach(config DEBUG MINSIZEREL RELEASE RELEASEWITHDEBINFO)
  if ("${config}" IN_LIST "RELEASE;MINSIZEREL")
    # note: -ffunction-sections impedes the MPLAB 8 debugger, so we only add it for non-debug builds
    string(APPEND CMAKE_C_FLAGS_${config}_INIT " -ffunction-sections")
    string(APPEND CMAKE_EXE_LINKER_FLAGS_${config}_INIT " --gc-sections")
  else()
    string(APPEND CMAKE_C_FLAGS_${config}_INIT " -g")
    set(CMAKE_EXE_LINKER_FLAGS_${config}_INIT "-Wl,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PK3=1,--defsym=__ICD2RAM=1")
  endif()
endforeach(config)

#  things I'm trying to get debug working:
##set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--defsym=__ICD2RAM=1"
##  #__ICD2RAM is used in the linker script to provision RAM for the PICKit3. We want to keep this so we can debug a live build
##  )


# set (CMAKE_LINKER "${XC16_ld_EXECUTABLE}")
