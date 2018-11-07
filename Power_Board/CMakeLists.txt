cmake_minimum_required(VERSION 3.5)
project(Power_Board)

set(CMAKE_C_STANDARD 90)

SET (C30 "/mnt/c/Program Files (x86)/Microchip/MPLAB C30")
SET ( CMAKE_CROSSCOMPILING ON )

SET (CMAKE_C_COMPILER clang-6.0)
SET (CMAKE_SYSTEM_PROCESSOR p24FJ256GB106)
# SET (CMAKE_C_COMPILER "${C30}/bin/pic30-gcc.exe")
#SET (CMAKE_C_LINK_EXECUTABLE  "${C30}/bin/pic30-ld.exe")
# set (HX pic32-bin2hex.exe)

add_definitions(-D__PIC24FJ256GB106__)
SET (CMAKE_C_FLAGS -mcpu=24FJ256GB106 -x c -g -Wall)
SET (CMAKE_EXE_LINKER_FLAGS  -Tp24FJ256GB106.gld)

include_directories(.)
include_directories("${C30}/include")
include_directories("${C30}/support/PIC24F/h")

include_directories(microchip)
include_directories(include)

add_executable(Power_Board
        closed_loop_control/core/InputCapture.c
        closed_loop_control/Filters.c
        closed_loop_control/Filters.h
        closed_loop_control/PID.c
        closed_loop_control/PID.h
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
        microchip/Compiler.h
        microchip/uart2.h
        src/debug_uart.c
        src/debug_uart.h
        "src/DEE Emulation 16-bit.c"
        "src/DEE Emulation 16-bit.h"
        src/device_robot_motor.c
        src/device_robot_motor.h
        src/device_robot_motor_i2c.c
        src/device_robot_motor_i2c.h
        src/device_robot_motor_loop.c
        src/device_robot_motor_loop.h
        "src/Flash Operations.s"
        src/i2clib.c
        src/i2clib.h
        src/interrupt_switch.c
        src/interrupt_switch.h
        src/main.c
        src/stdfunctions.c
        src/stdhdr.h
        src/testing.c
        src/testing.h
        include/device_robot_motor.h
        src/usb_descriptors.c
        src/usb_device.c
        HardwareProfile.h
        registers.h
        usb_config.c
        usb_config.h)
