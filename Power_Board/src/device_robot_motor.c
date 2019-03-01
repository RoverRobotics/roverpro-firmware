/// @file high-level motor control and main event loop.

#include "main.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "device_robot_motor_i2c.h"
#include "i2clib.h"
#include "motor.h"
#include "uart_control.h"
#include "power_bringup.h"
#include "power.h"
#include "math.h"
#include "flipper.h"
#include "drive.h"
#include "analog.h"
#include "cooling.h"

//****************************************************

void set_firmware_build_time(void);

//*********************************************//

/// Increment `value` until it hits limit,
/// at which point reset it to zero.
bool tick_counter(uint16_t *value, uint16_t limit) {
    if (++(*value) >= limit) {
        *value = 0;
        return true;
    } else {
        return false;
    }
}

void DeviceRobotMotorInit() {
    // make sure we start off in a default state
    AD1PCFGL = 0xffff;
    AD1PCFGH = 0x0003;

    TRISB = 0xffff;
    TRISC = 0xffff;
    TRISD = 0xffff;
    TRISE = 0xffff;
    TRISF = 0xffff;
    TRISG = 0xffff;

    init_power();

    uart_init();

    analog_init();

    // initialize motor control
    MotorsInit();
    // initialize motor tachometer feedback
    motor_tach_init();

    set_firmware_build_time();

    cooling_blast_fan();
}

void Device_MotorController_Process() {
    static struct {
        uint16_t power;
        uint16_t i2c;
        uint16_t communication;
        uint16_t analog;
        uint16_t flipper;
        uint16_t drive;
    } counters = {0};

    if (tick_counter(&counters.drive, g_settings.main.drive_poll_ms)) {
        drive_tick();
    }
    if (tick_counter(&counters.analog, g_settings.main.analog_poll_ms)) {
        analog_tick();
    }
    if (tick_counter(&counters.power, g_settings.main.power_poll_ms)) {
        power_tick();
    }
    if (tick_counter(&counters.i2c, g_settings.main.i2c_poll_ms)) {
        i2c_tick_all();
    }
    if (tick_counter(&counters.communication, g_settings.main.communication_poll_ms)) {
        uart_tick();
    }
    if (tick_counter(&counters.flipper, g_settings.main.flipper_poll_ms)) {
        tick_flipper_feedback();
    }
}

//**********************************************

void set_firmware_build_time(void) {
    const unsigned char build_date[12] = __DATE__;
    const unsigned char build_time[12] = __TIME__;
    unsigned int i;

    for (i = 0; i < 12; i++) {
        if (build_date[i] == 0) {
            REG_MOTOR_FIRMWARE_BUILD.data[i] = ' ';
        } else {
            REG_MOTOR_FIRMWARE_BUILD.data[i] = build_date[i];
        }
        if (build_time[i] == 0) {
            REG_MOTOR_FIRMWARE_BUILD.data[i + 12] = ' ';
        } else {
            REG_MOTOR_FIRMWARE_BUILD.data[i + 12] = build_time[i];
        }
    }
}
