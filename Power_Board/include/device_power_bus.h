#ifndef POWER_BOARD_DEVICE_POWER_BUS_H
#define POWER_BOARD_DEVICE_POWER_BUS_H

// Main power bus MOSFET control pins
#define CELL_A_MOS_EN(a) _TRISD3 = !(a)
#define CELL_B_MOS_EN(a) _TRISD2 = !(a)

#define Cell_A_MOS _LATD3
#define Cell_B_MOS _LATD2

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

// Power bus:
/// Activate batteries. Must communicate with batteries to figure out how to properly bring them up.
void power_bus_init();

/// When robot is on the charger, only leave one side of the power bus on at a time.
/// This is so that one side of a battery doesn't charge the other side through the power bus.
void power_bus_tick();

#endif // POWER_BOARD_DEVICE_POWER_BUS_H
