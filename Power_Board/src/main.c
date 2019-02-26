/// @file
/// Robotex one-size-fits-all firmware. Designed for the PIC24FJ256GB106 only.

// Set configuration bits.
// Note we can't overwrite these with the bootloader, so this is only needed
// for writing firmware with the PICkit3
#include "../../bootypic/devices/pic24fj256gb106/config.h"

#include "main.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "motor.h"
#include "settings.h"

Settings g_settings;

#define REGISTER_START()
#define REGISTER(a, b, c, d, e) e a;
#define REGISTER_END()
#include "registers.h"
#undef REGISTER_START
#undef REGISTER
#undef REGISTER_END

int main(void) {
    g_settings = settings_load();

    DeviceRobotMotorInit();

    while (1) {
        __builtin_clrwdt();
        Device_MotorController_Process();
    }
}
