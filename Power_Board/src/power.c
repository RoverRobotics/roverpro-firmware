#include "main.h"
#include "hardware_definitions.h"
#include "power.h"
#include "battery.h"
#include "i2clib.h"
#include "string.h"

extern void drive_set_coast_lock(bool is_on);

/// Activation routine for some batteries
static void turn_on_power_bus_old_method();
/// Activation routine for some batteries
static void turn_on_power_bus_new_method();
/// Activation routine for some batteries
static void turn_on_power_bus_hybrid_method();

static void turn_on_power_bus_immediate() { set_active_batteries(BATTERY_FLAG_ALL); }

static void turn_on_power_bus_new_method(void) {
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int k = 2000;
    unsigned int k0 = 2000;

    for (i = 0; i < 300; i++) {
        set_active_batteries(BATTERY_FLAG_ALL);
        for (j = 0; j < k; j++)
            __builtin_nop();
        set_active_batteries(BATTERY_FLAG_NONE);

        block_ms(10);

        k = k0 + i * i / 4;
    }

    set_active_batteries(BATTERY_FLAG_ALL);
}

static void turn_on_power_bus_old_method(void) {
    unsigned int i;

    for (i = 0; i < 20; i++) {
        set_active_batteries(BATTERY_FLAG_ALL);
        block_ms(10);
        set_active_batteries(BATTERY_FLAG_NONE);
        block_ms(40);
    }
    set_active_batteries(BATTERY_FLAG_ALL);
}

static void turn_on_power_bus_hybrid_method(void) {
    unsigned int i;
    unsigned int j;
    // k=20,000 is about 15ms
    unsigned int k = 2000;
    unsigned int k0 = 2000;

    for (i = 0; i < 200; i++) {
        set_active_batteries(BATTERY_FLAG_ALL);
        for (j = 0; j < k; j++)
            Nop();
        set_active_batteries(BATTERY_FLAG_NONE);
        block_ms(40);
        k = k0 + i * i / 4;
        if (k > 20000)
            k = 20000;
    }

    set_active_batteries(BATTERY_FLAG_ALL);
}

#define BATTERY_DATA_LEN 10
const char DEVICE_NAME_OLD_BATTERY[] = "BB-2590";
const char DEVICE_NAME_NEW_BATTERY[] = "BT-70791B";
const char DEVICE_NAME_BT70791_CK[] = "BT-70791CK";
const char DEVICE_NAME_CUSTOM_BATTERY[] = "ROBOTEX";

void init_power() {
    init_battery_io();

    // if the power bus is already active (like the bootloader did it)
    // then nothing to do here.
    if (get_active_batteries() == BATTERY_FLAG_ALL) {
        RCON = 0;
        return;
    }

    // _POR = "we are powering on from a black or brownout"
    // _EXTR = "our reset pin was hit"
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

// The battery has an power protection feature that will kill power if we draw too much current.
// So if we see the current spike, we kill the motors in order to prevent this.
static void power_tick_discharging() {
    static uint16_t current_spike_counter;
    static uint16_t current_recover_counter;

    // Not charging. Use both batteries.
    set_active_batteries(BATTERY_FLAG_ALL);

    uint16_t trigger_thresh = g_settings.power.overcurrent_trigger_threshold_ma * 34 / 1000;
    uint16_t reset_thresh = g_settings.power.overcurrent_reset_threshold_ma * 34 / 1000;

    if (g_state.analog.battery_current[0] >= trigger_thresh ||
        g_state.analog.battery_current[1] >= trigger_thresh) {
        // current spike
        current_spike_counter++;
        current_recover_counter = 0;

        if (current_spike_counter * g_settings.main.power_poll_ms >=
            g_settings.power.overcurrent_trigger_duration_ms) {
            g_state.power.overcurrent = true;
        }
    } else {
        current_recover_counter++;
        if (g_state.analog.battery_current[0] <= reset_thresh &&
            g_state.analog.battery_current[1] <= reset_thresh) {
            // low current state
            current_spike_counter = 0;
        }
        if (current_recover_counter * g_settings.main.power_poll_ms >=
            g_settings.power.overcurrent_reset_duration_ms) {
            g_state.power.overcurrent = false;
        }
    }
}

/// When robot is on the charger, only leave one side of the power bus on at a time.
/// This is so that one side of a battery doesn't charge the other side through the power bus.
static void power_tick_charging() {
    static uint16_t active_battery = BATTERY_FLAG_A;
    static uint16_t tick_charging = 0;

    if (++tick_charging * g_settings.main.power_poll_ms >
        g_settings.power.charging_battery_switch_ms) {
        // Toggle active battery
        active_battery = (active_battery == BATTERY_FLAG_A ? BATTERY_FLAG_B : BATTERY_FLAG_A);
        tick_charging = 0;
    }

    set_active_batteries(active_battery);
}

void power_tick() {
    if (g_state.i2c.charger_state == 0xdada) {
        power_tick_charging();
    } else {
        power_tick_discharging();
    }
}
