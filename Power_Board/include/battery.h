#ifndef BATTERY_H
#define BATTERY_H
#include "stdbool.h"

typedef enum BatteryFlag {
    BATTERY_NONE = 0,
    BATTERY_A = 1,
    BATTERY_B = 2,
    BATTERY_ALL = BATTERY_A | BATTERY_B,
} BatteryFlag;

void init_battery_io();

/// Set the power bus to draw from the specified batteries
void set_active_batteries(BatteryFlag state);

/// Get which battery/batteries we are currently drawing from
BatteryFlag get_active_batteries();

#endif