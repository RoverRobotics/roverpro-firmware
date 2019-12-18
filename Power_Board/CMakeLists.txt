cmake_minimum_required(VERSION 3.5)
project(PowerBoard C)

set(LINKER_SCRIPT "C:\\Users\\dan\\Documents\\OpenRoverFirmware\\bootypic\\devices\\pic24fj256gb106\\p24FJ256GB106_app.gld")
string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--script=${LINKER_SCRIPT},--heap=2048,-Map=${CMAKE_CURRENT_BINARY_DIR}/PowerBoard.map")

find_package(Python 3.7 REQUIRED)
configure_file(prebuild.py prebuild.py)
add_custom_command(
  OUTPUT src/version.GENERATED.h
  COMMAND "${Python_EXECUTABLE}" ${CMAKE_directory}prebuild.py src/version.GENERATED.h
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS prebuild.py
)

file(GLOB HEADERS ${PROJECT_SOURCE_DIR}/**.h)
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
  src/version.GENERATED.h
  ${HEADERS}
)

add_custom_command(
  TARGET Power_Board POST_BUILD
  COMMAND "${XC16}/bin/xc16-bin2hex.exe" -omf=${CMAKE_EXECUTABLE_FORMAT} "${CMAKE_CURRENT_BINARY_DIR}/Power_Board.elf"
  BYPRODUCTS Power_Board.hex
)