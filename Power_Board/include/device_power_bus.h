#ifndef DEVICE_POWER_BUS_H
#define DEVICE_POWER_BUS_H

// Main power bus MOSFET control pins
#define CELL_A_MOS_EN(a) _TRISD3 = !(a)
#define CELL_B_MOS_EN(a) _TRISD2 = !(a)

#define Cell_A_MOS _RD3
#define Cell_B_MOS _RD2

// constant for special usage
typedef enum BatteryState {
    CELL_OFF = 0,
    CELL_ON = 1,
} BatteryState;
typedef enum BatteryChannel {
    CELL_A = 0,
    CELL_B = 1,
} BatteryChannel;
#define BATTERY_CHANNEL_COUNT 2

/// Activate batteries. Must communicate with batteries to figure out how to properly bring them up.
/// Note that there are 2 power buses: The "digital bus" which powers this microcontroller and we
/// can't turn off and the main power bus, which powers the motor and peripherals. The batteries
/// power *both*, so if a hardware overcurrent condition is triggered, we might forcibly reboot.
void power_bus_init();

/// Set a given battery to either power the power bus or not. Assumes power_bus_init has already
/// been called.
void set_battery_state(BatteryChannel Channel, BatteryState state);

/// When robot is on the charger, only leave one side of the power bus on at a time.
/// This is so that one side of a battery doesn't charge the other side through the power bus.
void power_bus_tick();

#endif