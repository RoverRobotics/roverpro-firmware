cmake_minimum_required(VERSION 3.5)
project(PowerBoard C)

set(LINKER_SCRIPT "C:\\Users\\dan\\Documents\\OpenRoverFirmware\\bootypic\\devices\\pic24fj256gb106\\p24FJ256GB106_app.gld")

find_package(Python 3.7 REQUIRED)

set(GENERATED_SRC_DIR "${CMAKE_CURRENT_BINARY_DIR}/src")
set(VERSION_HEADER_FILE "${GENERATED_SRC_DIR}/version.GENERATED.h")

add_custom_command(
  OUTPUT "${VERSION_HEADER_FILE}"
  COMMAND "${Python_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/prebuild.py" "${VERSION_HEADER_FILE}"
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
  ${VERSION_HEADER_FILE}
  ${HEADERS}
  )
target_include_directories(Power_Board PRIVATE "${GENERATED_SRC_DIR}")

target_link_options(Power_Board
  PRIVATE "LINKER:--script=${LINKER_SCRIPT},--heap=2048,-Map=${CMAKE_CURRENT_BINARY_DIR}/PowerBoard.map"
  )

add_custom_command(
  TARGET Power_Board POST_BUILD
  COMMAND "${XC16}/bin/xc16-bin2hex.exe" -omf=${CMAKE_EXECUTABLE_FORMAT} "${CMAKE_CURRENT_BINARY_DIR}/Power_Board.elf"
  BYPRODUCTS Power_Board.hex
)