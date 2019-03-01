#include "main.h"
#include "power.h"
#include "battery.h"
#include "i2clib.h"
#include "string.h"
#include "drive.h"

// The battery has an power protection feature that will kill power if we draw too much current.
// So if we see the current spike, we kill the motors in order to prevent this.
static void power_tick_discharging() {
    static uint16_t current_spike_counter;
    static uint16_t current_recover_counter;

    // Not charging. Use both batteries.
    set_active_batteries(BATTERY_ALL);

    uint16_t trigger_thresh = g_settings.power.overcurrent_trigger_threshold_ma * 34 / 1000;
    uint16_t reset_thresh = g_settings.power.overcurrent_reset_threshold_ma * 34 / 1000;

    if (REG_PWR_A_CURRENT >= trigger_thresh || REG_PWR_B_CURRENT >= trigger_thresh) {
        // current spike
        current_spike_counter++;
        current_recover_counter = 0;

        if (current_spike_counter * g_settings.main.power_poll_ms >=
            g_settings.power.overcurrent_trigger_duration_ms) {
            drive_set_coast_lock(true);
            g_overcurrent = true;
        }
    } else {
        current_recover_counter++;
        if (REG_PWR_A_CURRENT <= reset_thresh && REG_PWR_B_CURRENT <= reset_thresh) {
            // low current state
            current_spike_counter = 0;
        }
        if (current_recover_counter * g_settings.main.power_poll_ms >=
            g_settings.power.overcurrent_reset_duration_ms)
            drive_set_coast_lock(false);
        g_overcurrent = false;
    }
}

/// When robot is on the charger, only leave one side of the power bus on at a time.
/// This is so that one side of a battery doesn't charge the other side through the power bus.
static void power_tick_charging() {
    static uint16_t active_battery = BATTERY_A;
    static uint16_t tick_charging = 0;

    if (++tick_charging * g_settings.main.power_poll_ms >
        g_settings.power.charging_battery_switch_ms) {
        // Toggle active battery
        active_battery = (active_battery == BATTERY_A ? BATTERY_B : BATTERY_A);
        tick_charging = 0;
    }

    set_active_batteries(active_battery);
}

void power_tick() {
    if (REG_MOTOR_CHARGER_STATE == 0xdada) {
        power_tick_charging();
    } else {
        power_tick_discharging();
    }
}
