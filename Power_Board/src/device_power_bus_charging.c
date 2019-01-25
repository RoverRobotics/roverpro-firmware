#include "usb_config.h"
#include "counter.h"
#include "device_power_bus.h"

void power_bus_tick() {
    static BatteryChannel active_battery = CELL_A;
    static Counter counter = {.max = 10000};

    if (REG_MOTOR_CHARGER_STATE == 0xdada) {
        if (counter_tick(&counter) == COUNTER_EXPIRED) {
            // Toggle active battery
            active_battery = (active_battery == CELL_A ? CELL_B : CELL_A);
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
        // Not charging. use both batteries
        set_battery_state(CELL_B, CELL_ON);
        set_battery_state(CELL_A, CELL_OFF);
    }
}