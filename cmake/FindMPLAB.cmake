cmake_minimum_required(VERSION 3.12)

# this is pretty dumb and assumes you have set MPLAB_ROOT appropriately

find_program(MPLAB_pk3cmd_EXECUTABLE
  NAMES PK3CMD
  PATH_SUFFIXES "Programmer Utilities/PICkit3")


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MPLAB
  REQUIRED_VARS
    MPLAB_pk3cmd_EXECUTABLE
)