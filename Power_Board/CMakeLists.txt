cmake_minimum_required(VERSION 3.5)
project(Power_Board)

set(CMAKE_SYSTEM_INCLUDE_PATH /include)
set(CMAKE_SYSTEM_LIBRARY_PATH /lib)
set(CMAKE_SYSTEM_PROGRAM_PATH /bin)

if (APPLE)
    set(tools /Applications/microchip/xc16/v1.34)
endif(APPLE)

if (WIN32)
    #set(tools "C:/Program Files (x86)/Microchip/MPLAB C30/")
    set(tools "C:/Program Files (x86)/Microchip/xc16/v1.35/")
endif(WIN32)


# bypass CMake compiler detection
set(CMAKE_C_COMPILER_ID_RUN 1)
set(CMAKE_C_STANDARD_COMPUTED_DEFAULT 90)
set(CMAKE_C_STANDARD 90)
set(CMAKE_CROSSCOMPILING ON)

add_definitions(-D__PIC24F__ -D__PIC24FJ256GB106__)

include_directories(
        include
        microchip

        "${tools}/include"
        "${tools}/support/PIC24F/h"
        "${tools}/support/generic/h"
        "${tools}/support/peripheral_24F"
        )

add_executable(Power_Board
    src/Filters.c
    src/InputCapture.c
    src/PID.c
    src/device_robot_motor.c
    src/device_robot_motor_i2c.c
    src/device_robot_motor_loop.c
    src/i2clib.c
    src/interrupt_switch.c
    src/main.c
    src/stdfunctions.c
    src/usb_config.c
    src/usb_descriptors.c
    src/usb_device.c

    include/Filters.h
    include/HardwareProfile.h
    include/InputCapture.h
    include/PID.h
    include/device_robot_motor.h
    include/device_robot_motor_i2c.h
    include/device_robot_motor_loop.h
    include/i2clib.h
    include/interrupt_switch.h
    include/registers.h
    include/stdhdr.h
    include/usb_config.h

    microchip/Compiler.h
    "microchip/DEE Emulation 16-bit.c"
    microchip/USB/usb.h
    microchip/USB/usb_ch9.h
    microchip/USB/usb_common.h
    microchip/USB/usb_device.h
    microchip/USB/usb_function_audio.h
    microchip/USB/usb_function_ccid.h
    microchip/USB/usb_function_cdc.h
    microchip/USB/usb_function_generic.h
    microchip/USB/usb_function_hid.h
    microchip/USB/usb_function_midi.h
    microchip/USB/usb_function_msd.h
    microchip/USB/usb_hal.h
    microchip/USB/usb_hal_pic18.h
    microchip/USB/usb_hal_pic24.h
    microchip/USB/usb_hal_pic32.h
    microchip/USB/usb_host.h
    microchip/USB/usb_host_audio_v1.h
    microchip/USB/usb_host_cdc.h
    microchip/USB/usb_host_cdc_interface.h
    microchip/USB/usb_host_charger.h
    microchip/USB/usb_host_generic.h
    microchip/USB/usb_host_hid.h
    microchip/USB/usb_host_hid_parser.h
    microchip/USB/usb_host_msd.h
    microchip/USB/usb_host_msd_scsi.h
    microchip/USB/usb_host_printer.h
    microchip/USB/usb_host_printer_esc_pos.h
    microchip/USB/usb_host_printer_pcl_5.h
    microchip/USB/usb_host_printer_postscript.h
    microchip/USB/usb_host_printer_primitives.h
    microchip/USB/usb_otg.h
    microchip/USB/usb_printer_pos_bixolon_srp_270.h
    microchip/USB/usb_printer_pos_epson_tm_t88iv.h
    microchip/USB/usb_printer_pos_seiko_dpu_v445.h
    microchip/USB/usb_printer_pos_seiko_mpu_l465.h
    microchip/uart2.h)
