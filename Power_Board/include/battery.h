#ifndef BATTERY_H
#define BATTERY_H
#include "stdbool.h"

/// Which of the multiple batteries (for functions that may operate on either battery)
typedef enum BatteryChannel {
    CELL_A = 0, ///< Battery A
    CELL_B = 1, ///< Battery B
} BatteryChannel;

// TODO: should be a bool.
/// Whether a battery is attached to the power bus or not
typedef enum BatteryState {
    CELL_OFF = 0,
    CELL_ON = 1,
} BatteryState;

/// Set a given battery to either power the power bus or not.
void set_battery_state(BatteryChannel Channel, BatteryState state);

void init_battery_io();

/// @returns true if there is at least one battery powering the power bus
bool is_power_bus_energized();

#endif