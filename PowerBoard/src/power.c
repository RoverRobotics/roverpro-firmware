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

enum SMARTBATTERY_REGISTER {
	SB_REG_SPECIFICATION_INFO = 0x1A,
	SB_REG_BATTERY_STATUS = 0x16,
};

static bool try_turn_on_power_bus_adaptive() {
    int32_t pulse_on_us = 600;
    int32_t pulse_max_us = 10000;
	
	{
		I2CResult result[2] = {0};
		uint16_t spec_info[2] = {0};
		
	    result[0] = i2c_synchronously_await(
	        I2C_BUS2, i2c_op_read_word(BATTERY_ADDRESS, SB_REG_SPECIFICATION_INFO, &spec_info[0]));
	    result[1] = i2c_synchronously_await(
	        I2C_BUS3, i2c_op_read_word(BATTERY_ADDRESS, SB_REG_SPECIFICATION_INFO, &spec_info[1]));
		
		if (
			result[0] != I2C_OKAY || (spec_info[0] & 0xF) != 0x1 ||
			result[1] != I2C_OKAY || (spec_info[1] & 0xF) != 0x1
		)
		{
			// I2C communication isn't working, or the battery doesn't speak smartbattery
			return false;
		}
	}
	
    while (pulse_on_us < pulse_max_us) {
		I2CResult result[2] = {0};
        uint16_t battery_status[2] = {0};

        set_active_batteries(BATTERY_FLAG_ALL);
        block_us(pulse_on_us);
		set_active_batteries(BATTERY_FLAG_NONE);

        result[0] = i2c_synchronously_await(
            I2C_BUS2, i2c_op_read_word(BATTERY_ADDRESS, SB_REG_BATTERY_STATUS, &battery_status[0]));
        result[1] = i2c_synchronously_await(
            I2C_BUS3, i2c_op_read_word(BATTERY_ADDRESS, SB_REG_BATTERY_STATUS, &battery_status[1]));

		const uint16_t SB_FLAG_TERMINATE_DISCHARGE = 0x0800;
        if ((result[0] == I2C_OKAY && !(battery_status[0] & SB_FLAG_TERMINATE_DISCHARGE)) ||
            (result[1] == I2C_OKAY && !(battery_status[1] & SB_FLAG_TERMINATE_DISCHARGE))) {
            // either battery reports okay power draw
            __builtin_clrwdt(); 
            pulse_on_us += (pulse_on_us / 16);
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