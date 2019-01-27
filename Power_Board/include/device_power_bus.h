#ifndef DEVICE_POWER_BUS_H
#define DEVICE_POWER_BUS_H

// TODO: unify Cell_A_MOS and CELL_A_MOS_EN.
/// The pin that gates battery A. Set high to allow the battery to power the main bus.
#define Cell_A_MOS _RD3
/// The pin that gates battery B. Set high to allow the battery to power the main bus.
#define Cell_B_MOS _RD2

// TODO: should be a bool.
/// Whether a battery is attached to the power bus or not
typedef enum BatteryState {
    CELL_OFF = 0,
    CELL_ON = 1,
} BatteryState;

/// Which of the multiple batteries (for functions that may operate on either battery)
typedef enum BatteryChannel {
    CELL_A = 0, ///< Battery A
    CELL_B = 1, ///< Battery B
} BatteryChannel;
/// Number of batteries in the rover. Keep synced with BatteryChannel.
#define BATTERY_CHANNEL_COUNT 2

/// Brings both batteries up. Since the power bus may draw a lot of current at first and different
/// batteries have different overcurrent protection logic (which is proprietary, undocumented dark
/// magic), this function toggles the batteries to keep power draw reasonable. Note that there are 2
/// power buses: The "digital bus" (which powers this microcontroller and we can't turn off) and the
/// "main bus" (which powers the motor and payload). We are only dealing here with the main bus, but
/// a hardware overcurrent condition would kill power to both!
void power_bus_init();

/// Set a given battery to either power the power bus or not. Assumes power_bus_init has already
/// been called.
void set_battery_state(BatteryChannel Channel, BatteryState state);

/// When robot is on the charger, only leave one side of the power bus on at a time.
/// This is so that one side of a battery doesn't charge the other side through the power bus.
void power_bus_tick();

#endif