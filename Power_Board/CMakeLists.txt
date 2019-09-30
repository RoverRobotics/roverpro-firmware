cmake_minimum_required(VERSION 3.5)
project(Power_Board C ASM)

set(CMAKE_C_STANDARD 90)
set(CMAKE_SYSTEM_PROCESSOR 24FJ256GB106)

add_compile_options(
  -mcpu=24FJ256GB106
  -no-legacy-libc
)
include_directories(
  ${PROJECT_SOURCE_DIR}/src
)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../bootypic/devices/pic24fj256gb106/p24FJ256GB106_app.gld")
add_link_options(
  -T "${LINKER_SCRIPT}"
  --architecture=${CMAKE_SYSTEM_PROCESSOR}
)

file(GLOB HEADERS ${PROJECT_SOURCE_DIR}/**.h)

configure_file(prebuild.py prebuild.py)
add_custom_command(
  OUTPUT src/version.GENERATED.h
  COMMAND python3 ${CMAKE_directory}prebuild.py src/version.GENERATED.h
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS prebuild.py
)

add_executable(Power_Board
  src/analog.c
  src/battery.c
  src/bytequeue.c
  src/clock.c
  src/communication.c
  src/cooling.c
  src/drive.c
  src/flipper.c
  src/i2clib.c
  src/main.c
  src/motor.c
  src/poll_i2c.c
  src/power.c
  src/power2.c
  src/settings.c
  src/stdfunctions.c
  ../bootypic/devices/pic24fj256gb106/p24FJ256GB106_app.gld
  src/version.GENERATED.h
  ${HEADERS}
  )
set(CMAKE_EXECUTABLE_FORMAT elf)
set(CMAKE_EXECUTABLE_SUFFIX .elf)
