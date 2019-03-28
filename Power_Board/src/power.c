#include "battery.h"
#include "hardware_definitions.h"
#include "i2clib.h"
#include "power.h"
#include "stdhdr.h"
#include "string.h"

/// Turn all the batteries on immediately
static void turn_on_power_bus_immediate() { set_active_batteries(BATTERY_FLAG_ALL); }

/// Pulse the power bus with constant-length pulses
/// (10 ms on + 40 ms off) x 20
static void turn_on_power_bus_old_method(void) {
    uint32_t i;
    for (i = 0; i < 20; i++) {
        set_active_batteries(BATTERY_FLAG_ALL);
        block_ms(10);
        set_active_batteries(BATTERY_FLAG_NONE);
        block_ms(40);
    }
    set_active_batteries(BATTERY_FLAG_ALL);
}

/// Pulse the power bus with quadratic-length pulses
/// (250 + (i**2 / 32) microseconds on + 10 ms off) x 300
static void turn_on_power_bus_new_method() {
    uint32_t i = 0;
    for (i = 0; i < 300; i++) {
        set_active_batteries(BATTERY_FLAG_ALL);
        block_us(250 + i * i / 32);
        set_active_batteries(BATTERY_FLAG_NONE);
        block_us(10000);
    }
    set_active_batteries(BATTERY_FLAG_ALL);
}

/// Pulse the power bus with quadratic-length pulses, capping the pulses at 1.25 ms
/// (250 + (i**2 / 32) microseconds on + 40 ms off) x 200
static void turn_on_power_bus_hybrid_method() {
    uint32_t i;
    for (i = 0; i < 200; i++) {
        set_active_batteries(BATTERY_FLAG_ALL);
        block_us(250 + i * i / 32);
        set_active_batteries(BATTERY_FLAG_NONE);
        block_us(40000);
    }
    set_active_batteries(BATTERY_FLAG_ALL);
}

#define BATTERY_DATA_LEN 10
const char DEVICE_NAME_BB2590[] = "BB-2590";
const char DEVICE_NAME_BT70791B[] = "BT-70791B";
const char DEVICE_NAME_BT70791CK[] = "BT-70791CK";

void power_init() {
    init_battery_io();

    // if the power bus is already active (like the bootloader did it)
    // then nothing to do here.
    if (get_active_batteries() == BATTERY_FLAG_ALL) {
        RCON = 0;
        return;
    }

    // POR = "we are powering on from a black or brownout"
    // EXTR = "our reset pin was hit"
    // If the system is "warm", assume the power bus is still energized and just switch the power
    // bus back on.
    if (!RCONbits.POR) {
        turn_on_power_bus_immediate();
        RCON = 0;
        return;
    }

    // clear the restart reason
    RCON = 0;
    char battery_data1[BATTERY_DATA_LEN] = {0};
    char battery_data2[BATTERY_DATA_LEN] = {0};
    I2CResult result;
    I2COperationDef op;

    // initialize i2c buses
    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);

    int j;
    for (j = 0; j < 3; j++) {
        // Read "Device Name" from battery
        op = i2c_op_read_block(BATTERY_ADDRESS, 0x21, battery_data1, BATTERY_DATA_LEN);
        result = i2c_synchronously_await(I2C_BUS2, op);
        if (result == I2C_OKAY)
            break;
        op = i2c_op_read_block(BATTERY_ADDRESS, 0x21, battery_data2, BATTERY_DATA_LEN);
        result = i2c_synchronously_await(I2C_BUS3, op);
        if (result == I2C_OKAY)
            break;
    }
    // If we're using the old battery (BB-2590)
    if (strcmp(DEVICE_NAME_BB2590, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_BB2590, battery_data2) == 0) {
        turn_on_power_bus_old_method();
    }

    // If we're using the new battery (BT-70791B)
    else if (strcmp(DEVICE_NAME_BT70791B, battery_data1) == 0 ||
             strcmp(DEVICE_NAME_BT70791B, battery_data2) == 0) {
        turn_on_power_bus_new_method();
    }

    // If we're using Bren-Tronics BT-70791C
    else if (strcmp(DEVICE_NAME_BT70791CK, battery_data1) == 0 ||
             strcmp(DEVICE_NAME_BT70791CK, battery_data2) == 0) {
        turn_on_power_bus_new_method();
    }

    // if we're using an unknown battery
    else {
        turn_on_power_bus_hybrid_method();
    }
}