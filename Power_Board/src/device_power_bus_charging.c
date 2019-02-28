#include "main.h"
#include "device_power_bus.h"

void power_bus_tick() {
    static BatteryChannel active_battery = CELL_A;
    static uint16_t tick_charging = 0;

    if (REG_MOTOR_CHARGER_STATE == 0xdada) {
	    if (++tick_charging * g_settings.main.electrical_poll_ms > g_settings.electrical.charging_battery_switch_ms) {
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
    } else {
    	tick_charging = 0;
        // Not charging. use both batteries
        set_battery_state(CELL_B, CELL_ON);
        set_battery_state(CELL_A, CELL_ON);
    }
}