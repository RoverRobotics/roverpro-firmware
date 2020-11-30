cmake_minimum_required(VERSION 3.12)
project("Rover Pro Firmware" VERSION 1.11.1 LANGUAGES C)

set(BOOTYPIC_DEVICE_DIR "${CMAKE_CURRENT_LIST_DIR}/bootypic/devices/pic24fj256gb106")
set(BOOTYPIC_ROOT ${CMAKE_CURRENT_LIST_DIR}/bootypic)
set(POWERBOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/PowerBoard)

add_subdirectory(${BOOTYPIC_ROOT})
add_subdirectory(${POWERBOARD_ROOT})

function(bin2hex target filestem)
  set(bin2hex_staging "${filestem}.prebin2hex")
  set(hexfile "${filestem}.hex")

  add_custom_command(
    OUTPUT "${hexfile}"
    COMMAND cmake -E create_symlink "$<TARGET_FILE:${target}>" "${bin2hex_staging}"
    COMMAND "${XC16_bin2hex_EXECUTABLE}" "${bin2hex_staging}"
    BYPRODUCTS "${bin2hex_staging}"
    DEPENDS "${target}"
  )
  add_custom_target(
    "${target}_hex"
    DEPENDS "${hexfile}"
  )
  get_filename_component(hexfile_path "${hexfile}"
    ABSOLUTE BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")
  install(FILES "${hexfile_path}" DESTINATION ".")
endfunction()

bin2hex(bootypic bootypic)
bin2hex(PowerBoard powerboard)

set(boot bootypic.hex)
set(app powerboard.hex)
set(combined combined.hex)
find_program(HEXMERGE_EXECUTABLE NAMES hexmerge hexmerge.py)
add_custom_command(
  OUTPUT ${combined}
  COMMAND "${HEXMERGE_EXECUTABLE}" --output=${combined} ${boot}::7 ${app}:8:3ff ${boot}:800:3fff ${app}:4000:0x557F3 ${boot}:0x557F4:
  DEPENDS "${boot}" "${app}" "${HEXMERGE_EXECUTABLE}"
)
add_custom_target(Combined ALL DEPENDS ${combined})
get_filename_component(combined_hex_path "${combined}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")
install(FILES "${combined_hex_path}" DESTINATION .)

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
