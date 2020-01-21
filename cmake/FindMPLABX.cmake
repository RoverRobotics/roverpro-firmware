cmake_minimum_required(VERSION 3.12)

# requires installing MPLAB IPE https://microchipdeveloper.com/ipe:installation
# when installing make sure MPLAB IPE is installed with 16 bit MCU support

# this is pretty dumb and assumes you have set MPLABX_ROOT appropriately
find_program(MPLABX_ide_EXECUTABLE
  NAMES mplab_ide_64 mplab_ide
  PATH_SUFFIXES mplab_platform/bin)
find_program(MPLABX_ipe_EXECUTABLE
  NAMES mplab_ipe_64 mplab_ipe
  PATH_SUFFIXES mplab_platform/bin)
find_program(MPLABX_ipecmd_EXECUTABLE
  NAMES ipecmd
  PATH_SUFFIXES mplab_platform/mplab_ipe)
find_program(MPLABX_hexmate_EXECUTABLE
  NAMES hexmate
  PATH_SUFFIXES mplab_platform/bin)

execute_process(COMMAND MPLABX_ipecmd_EXECUTABLE "-?"
  OUTPUT_VARIABLE IPECMD_HELP_OUTPUT)

string(REGEX MATCH "Version=\"([0-9]*.[0-9]*)\"" _ "${IPECMD_HELP_OUTPUT}")
set(MPLABX_VERSION_FOUND "${CMAKE_MATCH_1}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MPLABX
  REQUIRED_VARS
    MPLABX_ide_EXECUTABLE
    MPLABX_ipe_EXECUTABLE
    MPLABX_ipecmd_EXECUTABLE
    MPLABX_hexmate_EXECUTABLE
  VERSION_VAR MPLABX_VERSION_FOUND
)