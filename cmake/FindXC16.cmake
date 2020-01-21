cmake_minimum_required(VERSION 3.12)

# this is pretty dumb and assumes you have set XC16_ROOT appropriately

find_file(XC16_LANGUAGE_TOOL_SUITE ".LanguageToolSuite"
  PATH_SUFFIXES "bin")

file(READ "${XC16_LANGUAGE_TOOL_SUITE}" tool_suite_text)

string(REGEX MATCH "lti:version=\"([0-9]*.[0-9]*)\"" _ "${tool_suite_text}")
set(XC16_VERSION_FOUND "${CMAKE_MATCH_1}")

foreach(program ar as bin2hex cc1 gcc ld nm objdump pa ranlib readelf strings strip)
  find_program("XC16_${program}_EXECUTABLE" "xc16-${program}" PATH_SUFFIXES "bin/")
endforeach()

foreach(omf elf coff)
  foreach(program ar as bin2hex cc1 gcc ld nm objdump pa ranlib readelf strings strip)
    find_program("XC16_${omf}-${program}_EXECUTABLE" "${omf}-${program}" PATH_SUFFIXES "bin/bin/")
  endforeach()
endforeach()

add_link_options(LINKER:--report-mem)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XC16
  REQUIRED_VARS XC16_gcc_EXECUTABLE XC16_elf-gcc_EXECUTABLE XC16_coff-gcc_EXECUTABLE
  VERSION_VAR XC16_VERSION_FOUND
  )
