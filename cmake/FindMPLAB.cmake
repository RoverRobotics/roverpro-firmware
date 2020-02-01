cmake_minimum_required(VERSION 3.12)

# this is pretty dumb and assumes you have set MPLAB_ROOT appropriately

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

find_file(MPLAB_IDE_VERSION_FILE "mplab_ide.versions"
  PATH_SUFFIXES mplab_platform/etc)

file(STRINGS ${MPLAB_IDE_VERSION_FILE}
  MPLAB_IDE_VERSION_LIST
  LIMIT_COUNT 1)

if ("${MPLAB_IDE_VERSION_LIST}" MATCHES "v([0-9\\.]+)")
  set (MPLABX_VERSION_FOUND "${CMAKE_MATCH_1}")
endif()

execute_process(COMMAND MPLABX_ipecmd_EXECUTABLE "-?"
  OUTPUT_VARIABLE IPECMD_HELP_OUTPUT)

#string(REGEX MATCH "Version=\"([0-9]*.[0-9]*)\"" _ "${IPECMD_HELP_OUTPUT}")
#set(MPLABX_VERSION_FOUND "${CMAKE_MATCH_1}")

function(MPLABX_ipecmd)
  include(CMakeParseArguments)
  cmake_parse_arguments(PARSE_ARGV 0 ARG
    "ERASE;BLANK_CHECK;RELEASE_RESET;POWER_TARGET"
    "TOOL;PART;READ;WRITE"
    "")

  if (NOT "${ARG_TOOL}")
    set(ARG_TOOL PICKIT3)
  endif()
  if (NOT "${ARG_PART}")
    set(ARG_PART 24FJ256GB106)
  endif()

  set(CMD_ARGS ${MPLABX_ipecmd_EXECUTABLE} -TP${ARG_TOOL} -P${ARG_PART})
  if (ARG_ERASE)
    list(APPEND CMD_ARGS -E)
  endif()
  if (ARG_BLANK_CHECK)
    list(APPEND CMD_ARGS -C)
  endif()
  if(ARG_RELEASE_RESET)
    list(APPEND CMD_ARGS -OL)
  endif()
  if(ARG_POWER_TARGET)
    list(APPEND CMD_ARGS -W)
  endif()

  if(DEFINED ARG_READ)
    list(APPEND CMD_ARGS "-GF${ARG_READ}")
  endif()

  if(DEFINED ARG_WRITE)
    list(APPEND CMD_ARGS "-F${ARG_WRITE}" "-M")
  endif()

  message(ERROR ${CMD_ARGS})
  execute_process(COMMAND ${CMD_ARGS})
endfunction()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MPLABX
  REQUIRED_VARS
    MPLABX_ide_EXECUTABLE
    MPLABX_ipe_EXECUTABLE
    MPLABX_ipecmd_EXECUTABLE
    MPLABX_hexmate_EXECUTABLE
  VERSION_VAR MPLABX_VERSION_FOUND
)