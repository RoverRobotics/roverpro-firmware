#include "main.h"
#include "power.h"
#include "battery.h"
#include "i2clib.h"
#include "string.h"
#include "drive.h"

// The battery has an power protection feature that will kill power if we draw too much current.
// So if we see the current spike, we kill the motors in order to prevent this.
static void power_tick_charging() {
	 static uint16_t current_spike_counter;
        static uint16_t current_recover_counter;

        // Not charging. use both batteries
        set_battery_state(CELL_B, CELL_ON);
        set_battery_state(CELL_A, CELL_ON);

        uint16_t trigger_thresh =
            g_settings.power.overcurrent_trigger_threshold_ma * 34 / 1000;
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
static void power_tick_discharging() {
    static BatteryChannel active_battery = CELL_A;
    static uint16_t tick_charging = 0;

	    if (++tick_charging * g_settings.main.power_poll_ms > g_settings.power.charging_battery_switch_ms) {
            // Toggle active battery
            active_battery = (active_battery == CELL_A ? CELL_B : CELL_A);
            tick_charging = 0;
        }

        // if we're on the dock, turn off one side of the power bus
        switch (active_battery) {
        case CELL_A:
            set_battery_state(CELL_A, CELL_ON);
            set_battery_state(CELL_B, CELL_OFF);
            break;
        case CELL_B:
            set_battery_state(CELL_B, CELL_ON);
            set_battery_state(CELL_A, CELL_OFF);
            break;
        }

}

void power_tick(){
	 if (REG_MOTOR_CHARGER_STATE == 0xdada) {
	power_tick_charging();
	}else{
	power_tick_discharging();
	}
}	
