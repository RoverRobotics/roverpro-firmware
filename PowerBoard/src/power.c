#include "power.h"
#include "battery.h"
#include "hardware_definitions.h"
#include "i2clib.h"
#include "stdhdr.h"
#include <string.h>

/// Turn all the batteries on immediately
static bool try_turn_on_power_bus_immediate() {
    if (RCONbits.POR) {
        return false;
    }
    // If the system is "warm", assume the power bus is still energized and just switch the power
    // bus back on.
    set_active_batteries(BATTERY_FLAG_ALL);
    return true;
}

/// Pulse the power bus with quadratic-length pulses
/// (600 + (i**2 / 8) microseconds on + 10 ms off) x 300
static void turn_on_power_bus_new_method() {
    uint32_t i = 0;
    for (i = 0; i < 300; i++) {
        set_active_batteries(BATTERY_FLAG_ALL);
        block_us(600 + i * i / 8);
        set_active_batteries(BATTERY_FLAG_NONE);
        block_us(10000);
    }
    set_active_batteries(BATTERY_FLAG_ALL);
}

static bool try_turn_on_power_bus_adaptive() {
    int32_t pulse_on_us = 600;
    int32_t pulse_max_us = 10000;
    while (pulse_on_us < pulse_max_us) {
        set_active_batteries(BATTERY_FLAG_ALL);
        block_us(pulse_on_us);
        set_active_batteries(BATTERY_FLAG_NONE);
        I2CResult result[2] = {0};
        uint16_t battery_status[2] = {0};

        result[0] = i2c_synchronously_await(
            I2C_BUS2, i2c_op_read_word(BATTERY_ADDRESS, 0x16, &battery_status[0]));
        result[1] = i2c_synchronously_await(
            I2C_BUS3, i2c_op_read_word(BATTERY_ADDRESS, 0x16, &battery_status[1]));

        if (result[0] != I2C_OKAY && result[1] != I2C_OKAY) {
            // we can't communicate with either battery. Abort.
            return false;
        }
        if ((result[0] == I2C_OKAY && !(battery_status[0] & 0x0800)) ||
            (result[1] == I2C_OKAY && !(battery_status[1] & 0x0800))) {
            // either battery reports okay power draw
            __builtin_clrwdt();
            pulse_on_us = pulse_on_us * 9 / 8;
        }
    }
    set_active_batteries(BATTERY_FLAG_ALL);
    return true;
}

void power_init() {
    init_battery_io();

    // if the power bus is already active (like the bootloader did it)
    // then nothing to do here.
    if (get_active_batteries() == BATTERY_FLAG_ALL) {
        return;
    }

    // If the system is "warm", assume the power bus is still energized and just switch the power
    // bus back on.
    if (try_turn_on_power_bus_immediate()) {
        return;
    }
    if (try_turn_on_power_bus_adaptive()) {
        return;
    } else {
        turn_on_power_bus_new_method();
    }
}