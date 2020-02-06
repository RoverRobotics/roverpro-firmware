cmake_minimum_required(VERSION 3.12)

set(MICROCHIP_CPU_FAMILY PIC24F)
set(MICROCHIP_CPU_MODEL_NAME pic24fj256gb106)
set(MICROCHIP_CPU_ID 24fj256gb106)

set(CMAKE_SYSTEM_NAME Generic) # embedded, without OS
set(CMAKE_SYSTEM_VERSION 0)
set(CMAKE_SYSTEM_PROCESSOR ${MICROCHIP_CPU_ID})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

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

#tell all XC16 tools that we're using elf
set(ENV{XC16_OMF} elf)
set(CMAKE_EXECUTABLE_SUFFIX_C .elf)

# there are still some problems with compiler identification since XC16 mishandles the -v/--verbose flag
# setting the compiler to elf-gcc instead of xc16-gcc seems to work slightly better
set(CMAKE_C_COMPILER "${XC16_elf-gcc_EXECUTABLE}")
#set(CMAKE_C_COMPILER "${XC16_gcc_EXECUTABLE}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_LINKER_WRAPPER_FLAG "-Wl,")
set(CMAKE_C_LINKER_WRAPPER_FLAG_SEP ",")

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

set(CMAKE_C_FLAGS_DEBUG_INIT "-g")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG_INIT "-Wl,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--no-gc-sections")

# note: -ffunction-sections impedes the MPLAB 8 debugger, so we only add it for non-debug builds
set(CMAKE_C_FLAGS_MINSIZEREL_INIT "-Os -DNDEBUG -ffunction-sections")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL_INIT "-Wl,--gc-sections")

set(CMAKE_C_FLAGS_RELEASE_INIT "-O3 -ffunction-sections")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE_INIT "-Wl,--gc-sections")

set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO_INIT "-Wl,--no-gc-sections")


# these should be detected by compiler identification, but since they're not...
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES "${XC16_ROOT_DIR}/include" "${XC16_ROOT_DIR}/support/generic/h" )
set(CMAKE_C_IMPLICIT_LINK_DIRECTORIES "${XC16_ROOT_DIR}/lib")
set(CMAKE_C_IMPLICIT_LINK_LIBRARIES c pic30)
# I don't know if these are correct:
set(CMAKE_C_LIBRARY_ARCHITECTURE "pic24")
set(CMAKE_C_SIZEOF_DATA_PTR 2)
set(CMAKE_C_COMPILER_ABI ELF)

set(CMAKE_C_COMPILER_AR "${XC16_elf-ar_EXECUTABLE}" )
set(CMAKE_C_COMPILER_RANLIB "${XC16_elf-ranlib_EXECUTABLE}" )
