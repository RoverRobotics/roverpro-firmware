/**
 * @file main.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex one-size-fits-all firmware. Designed for the PIC24FJ256GB106 only.
 */

#include "stdhdr.h"
#include "device_robot_motor.h"
#include "motor.h"

// -------------------------------------------------------------------------
// PIC24FJ256GB106 FLASH CONFIGURATION
// -------------------------------------------------------------------------

// Set configuration bits.
// Note we can't overwrite these with the bootloader, so this is only needed
// for writing firmware with the PICkit3
#include "../../bootypic/devices/pic24fj256gb106/config.h"

// -------------------------------------------------------------------------
// BOOTLOADER
// -------------------------------------------------------------------------

// -------------------------------------------------------------------------
// GLOBAL VARIABLES
// -------------------------------------------------------------------------

int gpio_id = 0;
int gRegisterCount = 0;

// -------------------------------------------------------------------------
// CODE
// -------------------------------------------------------------------------

int main(void) {
    DeviceRobotMotorInit();

    while (1) {
        __builtin_clrwdt();
    	Device_MotorController_Process();
    }
}
