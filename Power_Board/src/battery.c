#include "battery.h"
#include "hardware_definitions.h"

void set_active_batteries(BatteryFlag state) {
    Cell_A_MOS = (state & BATTERY_FLAG_A) ? 1 : 0;
    Cell_B_MOS = (state & BATTERY_FLAG_B) ? 1 : 0;
}

void init_battery_io() {
    CELL_A_MOS_EN(1);
    CELL_B_MOS_EN(1);
}

BatteryFlag get_active_batteries() {
    BatteryFlag result = BATTERY_FLAG_NONE;
    if (Cell_A_MOS)
        result |= BATTERY_FLAG_A;
    if (Cell_B_MOS)
        result |= BATTERY_FLAG_B;
    return result;
}
