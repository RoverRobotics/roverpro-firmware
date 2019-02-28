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
bool g_overcurrent = false;

#define REGISTER_START()
#define REGISTER(a, b, c, d, e) e a;
#define REGISTER_END()
#include "registers.h"
#undef REGISTER_START
#undef REGISTER
#undef REGISTER_END

/// Set up and start Timer1: 1 kHz, no interrupts
void IniTimer1() {
    T1CON = 0x0000;         // clear register
    T1CONbits.TCKPS = 0b00; // 0b00 = 1:1 prescale
    TMR1 = 0;               // clear timer1 register
    uint16_t FREQUENCY_HZ = 1000;
    PR1 = (FCY / FREQUENCY_HZ) - 1;
    T1CONbits.TON = SET; // start timer
}

int main(void) {
    g_settings = settings_load();

    DeviceRobotMotorInit();
    IniTimer1();
    while (1) {
        __builtin_clrwdt();
        if (IFS0bits.T1IF == SET) {
            IFS0bits.T1IF = CLEAR;
            Device_MotorController_Process();
        }
    }
}
