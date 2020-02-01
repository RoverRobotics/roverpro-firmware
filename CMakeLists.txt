cmake_minimum_required(VERSION 3.12)
project(OpenRoverFirmware VERSION 1.9.3 LANGUAGES C)

set(BOOTYPIC_DEVICE_DIR "${CMAKE_CURRENT_LIST_DIR}/bootypic/devices/pic24fj256gb106")

add_subdirectory(bootypic)
add_subdirectory(PowerBoard)
#
#set(UNIFIED_HEX unified.hex)
#add_custom_target(${UNIFIED_HEX} ALL
#  COMMAND "${MPLABX_hexmate_EXECUTABLE}" "-O${UNIFIED_HEX}"
#  "${CMAKE_CURRENT_BINARY_DIR}/bootypic/bootypic.hex"
#  "r4-,${CMAKE_CURRENT_BINARY_DIR}/PowerBoard/PowerBoard.hex"
#  DEPENDS bootypic PowerBoard
#  COMMENT "Combining bootypic and PowerBoard hex files..."
#)

set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Firmware and Bootloader for OpenRover Power Board hardware")
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_VENDOR OpenRover)

include (CPack REQUIRED)

#because CPack makes a target named "package" which clion doesn't like picking up
add_custom_target(package_target
  COMMAND "${CMAKE_COMMAND}" --build . --target package
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Building package"
  DEPENDS bootypic PowerBoard docs
  VERBATIM
  )
