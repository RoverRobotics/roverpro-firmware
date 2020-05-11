cmake_minimum_required(VERSION 3.12)
project(PowerBoard VERSION 1.10.0 LANGUAGES C)

# assumes LINKER_SCRIPT
# but making this an OPTION breaks the script

set(GENERATED_SRC_DIR "${CMAKE_CURRENT_BINARY_DIR}/src")
set(VERSION_HEADER_FILE "${GENERATED_SRC_DIR}/version.GENERATED.h")

string(TIMESTAMP DATE "%Y-%m-%d")
string(TIMESTAMP TIME "%H:%M:%S")
configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/src/version.h.in"
  "${VERSION_HEADER_FILE}"
  ESCAPE_QUOTES
  @ONLY
)

add_executable(PowerBoard
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
  )

set(README_MD ${CMAKE_CURRENT_SOURCE_DIR}/../README.md)

target_include_directories(PowerBoard
  PRIVATE "${GENERATED_SRC_DIR}")

set(LINKER_SCRIPT "${BOOTYPIC_DEVICE_DIR}/app.gld")
set_target_properties(PowerBoard
  PROPERTIES
  LINK_DEPENDS "${LINKER_SCRIPT}"
  )

target_link_options(PowerBoard
  PRIVATE
  # linker script
  "LINKER:-D_LINK_APP,--data-init,--pack-data,--script=${LINKER_SCRIPT}"
  # remove unused code, save output for debugging
  "LINKER:-Map=${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_PROPERTY:NAME>.map"
  # add a heap for malloc
  "LINKER:--heap=2048"
  )

set(POWERBOARD_HEX "PowerBoard.hex")

add_custom_target(PowerBoardHex ALL
  DEPENDS "${POWERBOARD_HEX}"
)

add_custom_command(OUTPUT "${POWERBOARD_HEX}"
  COMMAND "${XC16_bin2hex_EXECUTABLE}" "$<TARGET_FILE:PowerBoard>"
  MAIN_DEPENDENCY "$<TARGET_FILE:PowerBoard>"
)

install(FILES PowerBoardHex DESTINATION "")

find_package(Doxygen)
set(DOXYGEN_PROJECT_NAME "OpenRover Power Board Firmware")
set(DOXYGEN_MULTILINE_CPP_IS_BRIEF YES)
set(DOXYGEN_OPTIMIZE_OUTPUT_FOR_C YES)
set(DOXYGEN_EXTRACT_ALL YES)
set(DOXYGEN_HIDE_SCOPE_NAMES YES)
set(DOXYGEN_GENERATE_DEPRECATEDLIST NO)
set(DOXYGEN_LAYOUT_FILE "docsrc/layout.xml")
set(DOXYGEN_EXCLUDE "src/xc16_clang_tools_stubs.h")
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ${README_MD})
set(DOXYGEN_STRIP_CODE_COMMENTS NO)
set(DOXYGEN_ALPHABETICAL_INDEX NO)
set(DOXYGEN_GENERATE_TREEVIEW YES)
set(DOXYGEN_CALL_GRAPH YES)
set(DOXYGEN_CALLER_GRAPH YES)
set(DOXYGEN_MACRO_EXPANSION YES)
set(DOXYGEN_CLASS_DIAGRAMS YES)
set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/docs")

doxygen_add_docs(docs
  src
  ${README_MD}
  ALL)

install(DIRECTORY "${DOXYGEN_OUTPUT_DIRECTORY}" DESTINATION .)