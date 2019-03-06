/// @file
/// Low-level battery control functions

#ifndef BATTERY_H
#define BATTERY_H
#include "stdbool.h"

typedef enum Battery {
    BATTERY_A = 0,
    BATTERY_B = 1,
} Battery;
#define BATTERY_COUNT 2

typedef enum BatteryFlag {
    BATTERY_FLAG_NONE = 0,
    BATTERY_FLAG_A = 1 << BATTERY_A,
    BATTERY_FLAG_B = 1 << BATTERY_B,
    BATTERY_FLAG_ALL = BATTERY_FLAG_A | BATTERY_FLAG_B,
} BatteryFlag;

void init_battery_io();

/// Set the power bus to draw from the specified batteries
void set_active_batteries(BatteryFlag state);

/// Get which battery/batteries we are currently drawing from
BatteryFlag get_active_batteries();

#endif