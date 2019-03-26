cmake_minimum_required(VERSION 3.5)
project(Power_Board C ASM)

# set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_C_STANDARD 90)
set(CMAKE_SYSTEM_PROCESSOR 24FJ256GB106)
set(xc16 "C:/Program Files (x86)/Microchip/xc16/v1.36/")

add_compile_options(
        -mcpu=24FJ256GB106
        -no-legacy-libc
)
include_directories(
        ${PROJECT_SOURCE_DIR}/src
)
include_directories(SYSTEM
        "${xc16}/include"
        "${xc16}/support/PIC24F/h"
        )
link_directories(
        ${xc16}/lib/
)
set(LINKER_SCRIPT "C:/Users/dan/Documents/OpenRoverFirmware-dan/bootypic/devices/pic24fj256gb106/p24FJ256GB106_app.gld")
add_link_options(
        -T "${LINKER_SCRIPT}"
        --architecture=${CMAKE_SYSTEM_PROCESSOR}
)

file(GLOB HEADERS ${PROJECT_SOURCE_DIR}/**.h)

configure_file(prebuild.py prebuild.py)
add_custom_command(
        OUTPUT src/version.GENERATED.h
        COMMAND py ${CMAKE_directory}prebuild.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS prebuild.py
)

add_executable(Power_Board
        src/motor.c
        src/device_robot_motor.c
        src/polling_i2c.c
        src/i2clib.c
        src/main.c
        src/stdfunctions.c
        src/analog.c
        src/uart_control.c
        src/power.c
        src/power_bringup.c
        src/settings.c
        src/flipper.c
        ../bootypic/devices/pic24fj256gb106/p24FJ256GB106_app.gld
        src/version.GENERATED.h
        ${HEADERS}
)
set(CMAKE_EXECUTABLE_FORMAT elf)
set(CMAKE_EXECUTABLE_SUFFIX .elf)
