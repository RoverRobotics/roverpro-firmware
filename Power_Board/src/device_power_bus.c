#include "stdhdr.h"
#include "device_power_bus.h"
#include "hardware_definitions.h"
#include "i2clib.h"
#include "string.h"

/// Activation routine for some batteries
static void turn_on_power_bus_old_method();
/// Activation routine for some batteries
static void turn_on_power_bus_new_method();
/// Activation routine for some batteries
static void turn_on_power_bus_hybrid_method();

void set_battery_state(BatteryChannel Channel, BatteryState state) {
    switch (Channel) {
    case CELL_A:
        Cell_A_MOS = state;
        break;
    case CELL_B:
        Cell_B_MOS = state;
        break;
    }
}

static void turn_on_power_bus_immediate() {
    set_battery_state(CELL_A, CELL_ON);
    set_battery_state(CELL_B, CELL_ON);
}

static void turn_on_power_bus_new_method(void) {
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int k = 2000;
    unsigned int k0 = 2000;

    for (i = 0; i < 300; i++) {
        set_battery_state(CELL_A, CELL_ON);
        set_battery_state(CELL_B, CELL_ON);
        for (j = 0; j < k; j++)
            Nop();
        set_battery_state(CELL_A, CELL_OFF);
        set_battery_state(CELL_B, CELL_OFF);

        block_ms(10);

        k = k0 + i * i / 4;
    }

    set_battery_state(CELL_A, CELL_ON);
    set_battery_state(CELL_B, CELL_ON);
}

static void turn_on_power_bus_old_method(void) {
    unsigned int i;

    for (i = 0; i < 20; i++) {
        set_battery_state(CELL_A, CELL_ON);
        set_battery_state(CELL_B, CELL_ON);
        block_ms(10);
        set_battery_state(CELL_A, CELL_OFF);
        set_battery_state(CELL_B, CELL_OFF);
        block_ms(40);
    }

    set_battery_state(CELL_A, CELL_ON);
    set_battery_state(CELL_B, CELL_ON);
}

static void turn_on_power_bus_hybrid_method(void) {
    unsigned int i;
    unsigned int j;
    // k=20,000 is about 15ms
    unsigned int k = 2000;
    unsigned int k0 = 2000;

    for (i = 0; i < 200; i++) {
        set_battery_state(CELL_A, CELL_ON);
        set_battery_state(CELL_B, CELL_ON);
        for (j = 0; j < k; j++)
            Nop();
        set_battery_state(CELL_A, CELL_OFF);
        set_battery_state(CELL_B, CELL_OFF);

        block_ms(40);
        k = k0 + i * i / 4;
        if (k > 20000)
            k = 20000;
    }

    set_battery_state(CELL_A, CELL_ON);
    set_battery_state(CELL_B, CELL_ON);
}

#define BATTERY_DATA_LEN 10
const char DEVICE_NAME_OLD_BATTERY[] = "BB-2590";
const char DEVICE_NAME_NEW_BATTERY[] = "BT-70791B";
const char DEVICE_NAME_BT70791_CK[] = "BT-70791CK";
const char DEVICE_NAME_CUSTOM_BATTERY[] = "ROBOTEX";

void power_bus_init(void) {
    // if the power bus is already active (like the bootloader did it)
    // then nothing to do here.
    if (Cell_A_MOS == 1 && Cell_B_MOS == 1) {
        RCON = 0;
        return;
    }

    // _POR = "we are powering on from a black or brownout"
    // _EXTR = "our reset pin was hit""
    // If the system is "warm", we can just switch the power bus back on.
    if (!_POR && !_EXTR) {
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

    // enable outputs for power bus
    CELL_A_MOS_EN(1);
    CELL_B_MOS_EN(1);

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
    if (strcmp(DEVICE_NAME_OLD_BATTERY, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_OLD_BATTERY, battery_data2) == 0) {
        turn_on_power_bus_old_method();
        return;
    }

    // If we're using the new battery (BT-70791B)
    if (strcmp(DEVICE_NAME_NEW_BATTERY, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_NEW_BATTERY, battery_data2) == 0) {
        turn_on_power_bus_new_method();
        return;
    }

    // If we're using Bren-Tronics BT-70791C
    if (strcmp(DEVICE_NAME_BT70791_CK, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_BT70791_CK, battery_data2) == 0) {
        turn_on_power_bus_new_method();
        return;
    }

    // if we're using the low lithium custom Matthew's battery
    if (strcmp(DEVICE_NAME_CUSTOM_BATTERY, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_CUSTOM_BATTERY, battery_data2) == 0) {
        turn_on_power_bus_old_method();
        return;
    }

    // if we're using an unknown battery
    turn_on_power_bus_hybrid_method();
}
