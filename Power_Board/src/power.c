#ifndef BOOTYPIC
#include "main.h"
#endif

#include "hardware_definitions.h"
#include "power.h"
#include "battery.h"
#include "i2clib.h"
#include "string.h"

/// Activation routine for some batteries
static void turn_on_power_bus_old_method();
/// Activation routine for some batteries
static void turn_on_power_bus_new_method();
/// Activation routine for some batteries
static void turn_on_power_bus_hybrid_method();

static void turn_on_power_bus_immediate() { set_active_batteries(BATTERY_FLAG_ALL); }

/// Pulse the power bus with constant-length pulses
/// (10 ms on + 40 ms off) x 20
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

/// Pulse the power bus with quadric-length pulses
/// (250 + (i**2 / 32) microseconds on + 10 ms off) x 238
static void turn_on_power_bus_new_method() {
    uint16_t i = 0;
    // Note: i previously counted to 300, but I reduced it to 238 because
    // greater values would cause short pulses again so probably not needed
    for (i = 0; i < 238; i++) {
        set_active_batteries(BATTERY_FLAG_ALL);
        block_us(250 + i * i / 32);
        set_active_batteries(BATTERY_FLAG_NONE);
        block_us(10000);
    }
    set_active_batteries(BATTERY_FLAG_ALL);
}

/// Pulse the power bus with quadric-length pulses, capping the pulses at 1.25 ms
/// (250 + (i**2 / 32) microseconds on + 40 ms off) x 200
static void turn_on_power_bus_hybrid_method() {
    uint16_t i;
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

#ifndef BOOTYPIC
/// The battery has an power protection feature that will kill power if we draw too much current.
/// So if we see the current spike, we kill the motors in order to prevent this.
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
#endif