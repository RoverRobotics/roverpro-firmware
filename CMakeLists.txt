cmake_minimum_required(VERSION 3.12)
project("Rover Pro Firmware" VERSION 1.10.2 LANGUAGES C)

set(BOOTYPIC_DEVICE_DIR "${CMAKE_CURRENT_LIST_DIR}/bootypic/devices/pic24fj256gb106")
set(BOOTYPIC_ROOT ${CMAKE_CURRENT_LIST_DIR}/bootypic)
set(POWERBOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/PowerBoard)

add_subdirectory(${BOOTYPIC_ROOT})
add_subdirectory(${POWERBOARD_ROOT})

set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Firmware and Bootloader for Rover Pro Power Board hardware")
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_VENDOR "Rover Robotics")

include (CPack REQUIRED)

#because CPack makes a target named "package" which clion doesn't like picking up
add_custom_target(package_target
  COMMAND "${CMAKE_COMMAND}" --build . --target package
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Building package"
  DEPENDS bootypic PowerBoard docs
  VERBATIM
  )
