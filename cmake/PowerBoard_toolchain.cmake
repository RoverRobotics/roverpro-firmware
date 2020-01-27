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
set(CMAKE_EXECUTABLE_SUFFIX_C .elf)
set(CMAKE_EXECUTABLE_FORMAT ELF)

set(ENV{XC16_OMF} ${CMAKE_EXECUTABLE_FORMAT})

# while xc16-gcc.exe doesn't play nice with feature detection, coff-gcc and elf-gcc do.
# as per https://www.microchip.com/forums/m1099635.aspx
set(CMAKE_C_COMPILER "${XC16_elf-gcc_EXECUTABLE}")
set(CMAKE_C_STANDARD 99)
add_link_options(LINKER:-p${MICROCHIP_CPU_ID},--report-mem
  LINKER:--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PK3=1
  -mreserve=data@0x800:0x850)
#
#make -f nbproject/Makefile-XC16_24FJ256GA110.mk SUBPROJECTS= .build-conf
#make[1]: Entering directory 'C:/Users/dan/MPLABXProjects/pic24_c_template_1.X'
#make  -f nbproject/Makefile-XC16_24FJ256GA110.mk dist/XC16_24FJ256GA110/debug/pic24_c_template_1.X.debug.elf
#make[2]: Entering directory 'C:/Users/dan/MPLABXProjects/pic24_c_template_1.X'
#"C:\opt\Microchip\xc16\v1.41\bin\xc16-gcc.exe"   main.c  -o build/XC16_24FJ256GA110/debug/main.o  -c -mcpu=24FJ256GB106  -MMD -MF "build/XC16_24FJ256GA110/debug/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_XC16_24FJ256GA110=XC16_24FJ256GA110  -no-legacy-libc    -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="C:/opt/Microchip/MPLABX/v5.30/packs/Microchip/PIC24F-GA-GB_DFP/1.1.74"/xc16
#"C:\opt\Microchip\xc16\v1.41\bin\xc16-gcc.exe"   system.c  -o build/XC16_24FJ256GA110/debug/system.o  -c -mcpu=24FJ256GB106  -MMD -MF "build/XC16_24FJ256GA110/debug/system.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_XC16_24FJ256GA110=XC16_24FJ256GA110  -no-legacy-libc    -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="C:/opt/Microchip/MPLABX/v5.30/packs/Microchip/PIC24F-GA-GB_DFP/1.1.74"/xc16
#"C:\opt\Microchip\xc16\v1.41\bin\xc16-gcc.exe"   interrupts.c  -o build/XC16_24FJ256GA110/debug/interrupts.o  -c -mcpu=24FJ256GB106  -MMD -MF "build/XC16_24FJ256GA110/debug/interrupts.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_XC16_24FJ256GA110=XC16_24FJ256GA110  -no-legacy-libc    -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="C:/opt/Microchip/MPLABX/v5.30/packs/Microchip/PIC24F-GA-GB_DFP/1.1.74"/xc16
#"C:\opt\Microchip\xc16\v1.41\bin\xc16-gcc.exe"   configuration_bits.c  -o build/XC16_24FJ256GA110/debug/configuration_bits.o  -c -mcpu=24FJ256GB106  -MMD -MF "build/XC16_24FJ256GA110/debug/configuration_bits.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_XC16_24FJ256GA110=XC16_24FJ256GA110  -no-legacy-libc    -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="C:/opt/Microchip/MPLABX/v5.30/packs/Microchip/PIC24F-GA-GB_DFP/1.1.74"/xc16
#"C:\opt\Microchip\xc16\v1.41\bin\xc16-gcc.exe"   traps.c  -o build/XC16_24FJ256GA110/debug/traps.o  -c -mcpu=24FJ256GB106  -MMD -MF "build/XC16_24FJ256GA110/debug/traps.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_XC16_24FJ256GA110=XC16_24FJ256GA110  -no-legacy-libc    -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="C:/opt/Microchip/MPLABX/v5.30/packs/Microchip/PIC24F-GA-GB_DFP/1.1.74"/xc16
#"C:\opt\Microchip\xc16\v1.41\bin\xc16-gcc.exe"   user.c  -o build/XC16_24FJ256GA110/debug/user.o  -c -mcpu=24FJ256GB106  -MMD -MF "build/XC16_24FJ256GA110/debug/user.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_XC16_24FJ256GA110=XC16_24FJ256GA110  -no-legacy-libc    -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="C:/opt/Microchip/MPLABX/v5.30/packs/Microchip/PIC24F-GA-GB_DFP/1.1.74"/xc16
#"C:\opt\Microchip\xc16\v1.41\bin\xc16-gcc.exe"   -o dist/XC16_24FJ256GA110/debug/pic24_c_template_1.X.debug.elf  build/XC16_24FJ256GA110/debug/configuration_bits.o build/XC16_24FJ256GA110/debug/interrupts.o build/XC16_24FJ256GA110/debug/main.o build/XC16_24FJ256GA110/debug/system.o build/XC16_24FJ256GA110/debug/traps.o build/XC16_24FJ256GA110/debug/user.o      -mcpu=24FJ256GB106        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -DXPRJ_XC16_24FJ256GA110=XC16_24FJ256GA110  -no-legacy-libc     -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PK3=1,,--script=p24FJ256GB106.gld,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="dist/XC16_24FJ256GA110/debug/pic24_c_template_1.X.debug.map",--report-mem,--memorysummary,dist/XC16_24FJ256GA110/debug/memoryfile.xml  -mdfp="C:/opt/Microchip/MPLABX/v5.30/packs/Microchip/PIC24F-GA-GB_DFP/1.1.74"/xc16
#


# note: -mafrlcsj turns off license check
set(CMAKE_C_FLAGS_INIT " -no-legacy-libc -mafrlcsj -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mdfp=C:/opt/Microchip/MPLABX/v5.30/packs/Microchip/PIC24F-GA-GB_DFP/1.1.74/xc16")
set(CMAKE_C_FLAGS_DEBUG_INIT " -g -D__DEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL_INIT " -Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE_INIT " -O3 -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT " -O2 -g -DNDEBUG")

set(CMAKE_C_LINKER_WRAPPER_FLAG "-Wl,")
set(CMAKE_C_LINKER_WRAPPER_FLAG_SEP ",")
set(CMAKE_C_COMPILER "${XC16_elf-gcc_EXECUTABLE}")

add_compile_options(
  -std=c99
  -mcpu=${CMAKE_SYSTEM_PROCESSOR}
  # -mcci #todo: migrate to CCI
  -no-legacy-libc
  -ffunction-sections
  -fno-short-double
  -Wall
  -fast-math
)
