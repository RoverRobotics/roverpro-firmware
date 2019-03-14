/// @file
/// Handle the power supply. Prevents and warns of too-high current draw. Although the batteries
/// have protection circuitry to prevent damage, doing so will cause the power supply to black out
/// and this firmware (as well as any payload hardware) to lose power.

#ifndef POWER_H
#define POWER_H
/// Brings both batteries up. Since the power bus may draw a lot of current at first and different
/// batteries have different overcurrent protection logic (which is proprietary, undocumented dark
/// magic), this function toggles the batteries to keep power draw reasonable. Note that there are 2
/// power buses: The "digital bus" (which powers this microcontroller and we can't turn off) and the
/// "main bus" (which powers the motor and payload). We are only dealing here with the main bus,
/// but a hardware overcurrent condition would kill power to both!
void power_init();

#ifndef BOOTYPIC
/// Does an incremental amount of work to protect robot and the batteries.
/// - If charging, only connects one battery at a time to the main power bus
/// - If discharging, warns (via the overcurrent flag) if current draw is trending too high
void power_tick();
#endif

#endif