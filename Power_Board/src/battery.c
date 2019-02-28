#include "battery.h"
#include "hardware_definitions.h"

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

void init_battery_io() {
 	CELL_A_MOS_EN(1);
    CELL_B_MOS_EN(1);
}	

bool is_power_bus_energized() {
 	CELL_A_MOS_EN(1);
    CELL_B_MOS_EN(1);
	return (Cell_A_MOS == 1 || Cell_B_MOS == 1);
}